#!/usr/bin/env python3
"""Command-line interface for the Quoridor AI benchmark.

Usage examples
--------------
# Engine vs Engine (no model weights required):
python -m quoridor_benchmark \
    --engine depth=3 \
    --engine depth=1 \
    --games 20

# Engine vs DQN:
python -m quoridor_benchmark \
    --engine depth=3 \
    --dqn ~/quoridor_models/latest \
    --games 30 \
    --output ./benchmark_results

# Full three-way comparison:
python -m quoridor_benchmark \
    --engine depth=3 \
    --dqn ~/quoridor_models/latest \
    --alphazero ~/quoridor_az_models/latest \
    --az-sims 64 \
    --games 20 \
    --output ./benchmark_results

# AlphaZero vs specific DQNs only (skip DQN vs DQN):
python -m quoridor_benchmark \
    --alphazero ~/az_model,name=AlphaZero \
    --dqn ~/dqn1,name=DQN_4 \
    --dqn ~/dqn2,name=DQN_7 \
    --matchup AlphaZero:DQN_4 \
    --matchup AlphaZero:DQN_7 \
    --games 50 \
    --output ./benchmark_results
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

# Allow running as a script without installing.
sys.path.insert(0, str(Path(__file__).parent.parent))

from quoridor_benchmark.agents import (
    load_engine_agent,
    load_dqn_agent,
    load_alphazero_agent,
)
from quoridor_benchmark.benchmark import BenchmarkConfig, run_benchmark
from quoridor_benchmark.report import generate_report


def _parse_kv(s: str) -> dict[str, str]:
    """Parse 'key=val,key2=val2' into a dict."""
    result = {}
    for part in s.split(","):
        if "=" in part:
            k, v = part.split("=", 1)
            result[k.strip()] = v.strip()
    return result


def main(argv=None):
    p = argparse.ArgumentParser(
        description="Quoridor AI Benchmark — round-robin tournament",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )

    # Agent arguments
    p.add_argument(
        "--engine",
        metavar="depth=N[,noise=F]",
        action="append",
        default=[],
        help="Add a minimax engine agent. E.g. --engine depth=3",
    )
    p.add_argument(
        "--dqn",
        metavar="MODEL_DIR[,name=STR][,epsilon=F]",
        action="append",
        default=[],
        help="Add a DQN agent from a checkpoint directory.",
    )
    p.add_argument(
        "--alphazero",
        metavar="MODEL_DIR[,name=STR][,sims=N]",
        action="append",
        default=[],
        help="Add an AlphaZero agent from a checkpoint directory.",
    )

    # Tournament settings
    p.add_argument("--games",    type=int, default=20,   help="Games per ordered pair (default 20).")
    p.add_argument("--board-n",  type=int, default=5,    help="Board size (default 5).")
    p.add_argument("--max-plies",type=int, default=150,  help="Max plies per game (default 150).")
    p.add_argument("--seed",     type=int, default=42,   help="Random seed (default 42).")
    p.add_argument("--az-sims",  type=int, default=64,   help="MCTS sims for AlphaZero (default 64).")
    p.add_argument("--workers",  type=int, default=1,    help="Parallel workers (default 1; use 1 with TF).")
    p.add_argument("--output",   type=str, default=None, help="Directory to save report.txt, results.json, h2h.csv.")
    p.add_argument("--quiet",    action="store_true",    help="Suppress progress output.")
    p.add_argument(
        "--matchup",
        metavar="AGENT_A:AGENT_B",
        action="append",
        default=[],
        help=(
            "Restrict to specific matchups. Repeatable. "
            "Use agent names separated by ':'. "
            "If omitted, all pairs are tested (full round-robin). "
            "E.g. --matchup AlphaZero:DQN_4 --matchup AlphaZero:DQN_7"
        ),
    )

    args = p.parse_args(argv)

    agents: dict = {}

    # ── Engine agents ───────────────────────────────────────────────────────
    for spec in args.engine:
        kv = _parse_kv(spec)
        depth = int(kv.get("depth", 3))
        noise = float(kv.get("noise", 0.0))
        name  = kv.get("name", f"Engine-d{depth}")
        if name in agents:
            name += f"_{len(agents)}"
        agents[name] = load_engine_agent(depth=depth, noise=noise)
        print(f"  Registered: {name}  (minimax depth={depth}, noise={noise})")

    # ── DQN agents ──────────────────────────────────────────────────────────
    for spec in args.dqn:
        parts = spec.split(",")
        model_dir = parts[0] if "=" not in parts[0] else None
        kv = _parse_kv(",".join(parts[1:] if model_dir else parts))
        if model_dir is None:
            model_dir = kv.get("dir", ".")
        epsilon = float(kv.get("epsilon", 0.0))
        name    = kv.get("name", f"DQN")
        if name in agents:
            name += f"_{len(agents)}"
        agents[name] = load_dqn_agent(model_dir=model_dir, epsilon=epsilon)
        print(f"  Registered: {name}  (DQN from {model_dir}, eps={epsilon})")

    # ── AlphaZero agents ─────────────────────────────────────────────────────
    for spec in args.alphazero:
        parts = spec.split(",")
        model_dir = parts[0] if "=" not in parts[0] else None
        kv = _parse_kv(",".join(parts[1:] if model_dir else parts))
        if model_dir is None:
            model_dir = kv.get("dir", ".")
        sims  = int(kv.get("sims", args.az_sims))
        name  = kv.get("name", f"AlphaZero")
        if name in agents:
            name += f"_{len(agents)}"
        agents[name] = load_alphazero_agent(model_dir=model_dir, n_simulations=sims)
        print(f"  Registered: {name}  (AlphaZero from {model_dir}, sims={sims})")

    if len(agents) < 2:
        p.error("Need at least 2 agents. Use --engine, --dqn, --alphazero.")

    # ── Matchup filter ───────────────────────────────────────────────────────
    matchups = None
    if args.matchup:
        matchups = []
        for m in args.matchup:
            if ":" not in m:
                p.error(f"--matchup must be in format AGENT_A:AGENT_B, got: {m!r}")
            a, b = m.split(":", 1)
            a, b = a.strip(), b.strip()
            if a not in agents:
                p.error(f"--matchup agent '{a}' not registered. Check spelling and --dqn/--engine/--alphazero names.")
            if b not in agents:
                p.error(f"--matchup agent '{b}' not registered. Check spelling and --dqn/--engine/--alphazero names.")
            matchups.append((a, b))

    if matchups:
        print(f"\n  Running {len(matchups)} matchup(s) × {args.games} games/pair")
        for a, b in matchups:
            print(f"    {a} vs {b}")
    else:
        print(f"\n  Running round-robin: {len(agents)} agents × {args.games} games/pair")
    print()

    cfg = BenchmarkConfig(
        games_per_pair=args.games,
        board_n=args.board_n,
        max_plies=args.max_plies,
        seed=args.seed,
        n_workers=args.workers,
        verbose=not args.quiet,
        matchups=matchups,
    )

    result = run_benchmark(agents, config=cfg)
    print()
    generate_report(result, output_dir=args.output, print_report=True)


if __name__ == "__main__":
    main()
