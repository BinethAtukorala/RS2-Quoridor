#!/usr/bin/env python3
"""
Quick-start example — runs a benchmark with no model weights required.

Pits two minimax engines of different depths against each other so you
can verify everything works before plugging in DQN / AlphaZero weights.

Run from the quoridor_benchmark/ directory:
    python example_no_weights.py

Once you have trained weights, uncomment the DQN / AlphaZero blocks below.
"""
import sys
from pathlib import Path

# Make sure the package is importable when run from this directory.
sys.path.insert(0, str(Path(__file__).parent))

from quoridor_benchmark import run_benchmark, BenchmarkConfig, generate_report
from quoridor_benchmark.agents import (
    load_engine_agent,
    load_dqn_agent,          # needs weights
    load_alphazero_agent,    # needs weights
)

# ── Register agents ─────────────────────────────────────────────────────────
agents = {
    "Engine-d3": load_engine_agent(depth=3, noise=0.0),   # strong minimax
    "Engine-d1": load_engine_agent(depth=1, noise=0.0),   # weak minimax
    "Random":    load_engine_agent(depth=1, noise=1.0),   # fully random (noise=1)
}

# Uncomment and adjust paths when you have trained weights:
# agents["DQN"] = load_dqn_agent(
#     model_dir="/path/to/quoridor_models/latest",
#     epsilon=0.0,
# )
# agents["AlphaZero"] = load_alphazero_agent(
#     model_dir="/path/to/quoridor_az_models/latest",
#     n_simulations=64,
# )

# ── Run ──────────────────────────────────────────────────────────────────────
config = BenchmarkConfig(
    games_per_pair=30,      # ↑ for more stable Elo
    board_n=5,
    seed=42,
    verbose=True,
)

print("Starting benchmark...\n")
result = run_benchmark(agents, config=config)

# ── Report ───────────────────────────────────────────────────────────────────
generate_report(
    result,
    output_dir="./benchmark_output",  # saves report.txt / results.json / h2h.csv
    print_report=True,
)
