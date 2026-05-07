"""Core benchmark runner.

Runs a round-robin tournament between registered agents,
computes win rates, Elo ratings, and average game statistics.
"""
from __future__ import annotations

import itertools
import math
import random
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass, field
from typing import Callable

import numpy as np

from .match import play_match, MatchResult, Policy
from .quoridor_utils import QuoridorBoard


# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------

@dataclass
class BenchmarkConfig:
    """Configuration for a benchmark run."""
    games_per_pair: int = 20
    """Number of games per ordered pair (A vs B and B vs A each get this many)."""

    board_n: int = 5
    max_plies: int = 150
    seed: int | None = 42
    n_workers: int = 1
    """Parallel workers. Keep at 1 if any agent uses TensorFlow (not thread-safe)."""

    elo_k: float = 32.0
    elo_base: float = 1500.0

    verbose: bool = True


# ---------------------------------------------------------------------------
# Elo
# ---------------------------------------------------------------------------

def _expected(ra: float, rb: float) -> float:
    return 1.0 / (1.0 + 10.0 ** ((rb - ra) / 400.0))


def compute_elo(
    agent_names: list[str],
    results: list[MatchResult],
    k: float = 32.0,
    base: float = 1500.0,
    n_passes: int = 200,
) -> dict[str, float]:
    """Converged Elo via multiple shuffled passes with decaying K.

    Single-pass Elo is order-dependent and unstable for round-robin tournaments.
    Multiple passes over shuffled results with decaying K converges to stable ratings.
    """
    import random as _random
    elo = {name: base for name in agent_names}

    for pass_i in range(n_passes):
        k_eff = k * max(0.1, 1.0 - 0.9 * pass_i / max(1, n_passes - 1))
        shuffled = results[:]
        _random.shuffle(shuffled)

        for r in shuffled:
            if r.winner == "draw":
                sa, sb = 0.5, 0.5
            elif r.winner == "bot":
                sa, sb = 1.0, 0.0
            else:
                sa, sb = 0.0, 1.0

            ra, rb = elo[r.bot_name], elo[r.player_name]
            ea, eb = _expected(ra, rb), _expected(rb, ra)
            elo[r.bot_name]    += k_eff * (sa - ea)
            elo[r.player_name] += k_eff * (sb - eb)

    return elo


# ---------------------------------------------------------------------------
# Per-agent aggregate stats
# ---------------------------------------------------------------------------

@dataclass
class AgentStats:
    name: str
    wins: int = 0
    losses: int = 0
    draws: int = 0
    total_games: int = 0
    illegal_moves: int = 0
    total_plies_sum: int = 0
    move_time_ms: list[float] = field(default_factory=list)
    elo: float = 1500.0

    @property
    def win_rate(self) -> float:
        return self.wins / self.total_games if self.total_games else 0.0

    @property
    def avg_plies(self) -> float:
        return self.total_plies_sum / self.total_games if self.total_games else 0.0

    @property
    def avg_move_ms(self) -> float:
        return sum(self.move_time_ms) / len(self.move_time_ms) if self.move_time_ms else 0.0

    @property
    def max_move_ms(self) -> float:
        return max(self.move_time_ms) if self.move_time_ms else 0.0


def _update_stats(stats: dict[str, AgentStats], r: MatchResult):
    bot_s  = stats[r.bot_name]
    play_s = stats[r.player_name]

    bot_s.total_games  += 1
    play_s.total_games += 1
    bot_s.total_plies_sum  += r.plies
    play_s.total_plies_sum += r.plies

    bot_s.move_time_ms.extend(r.move_times_ms.get("bot", []))
    play_s.move_time_ms.extend(r.move_times_ms.get("player", []))

    if r.status == "illegal_move":
        loser = r.bot_name if r.winner == "player" else r.player_name
        stats[loser].illegal_moves += 1

    if r.winner == "bot":
        bot_s.wins   += 1
        play_s.losses += 1
    elif r.winner == "player":
        play_s.wins  += 1
        bot_s.losses  += 1
    else:
        bot_s.draws  += 1
        play_s.draws += 1


# ---------------------------------------------------------------------------
# Head-to-head record
# ---------------------------------------------------------------------------

@dataclass
class H2HRecord:
    wins_as_bot: int = 0
    wins_as_player: int = 0
    draws: int = 0
    games: int = 0

    @property
    def total_wins(self) -> int:
        return self.wins_as_bot + self.wins_as_player

    @property
    def win_rate(self) -> float:
        return self.total_wins / self.games if self.games else 0.0


# ---------------------------------------------------------------------------
# BenchmarkResult
# ---------------------------------------------------------------------------

@dataclass
class BenchmarkResult:
    config: BenchmarkConfig
    agent_names: list[str]
    all_matches: list[MatchResult]
    stats: dict[str, AgentStats]
    elo: dict[str, float]
    h2h: dict[tuple[str, str], H2HRecord]   # (A, B) = A's record against B

    @property
    def ranking(self) -> list[tuple[str, float]]:
        """Agents sorted by Elo, highest first."""
        return sorted(self.elo.items(), key=lambda x: x[1], reverse=True)


# ---------------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------------

def run_benchmark(
    agents: dict[str, Policy],
    config: BenchmarkConfig | None = None,
) -> BenchmarkResult:
    """Run a full round-robin benchmark.

    Parameters
    ----------
    agents : dict[str, Policy]
        name -> policy callable. At least 2 agents required.
    config : BenchmarkConfig | None
        Uses defaults if None.

    Returns
    -------
    BenchmarkResult with per-agent stats, Elo ratings, and head-to-head records.
    """
    if config is None:
        config = BenchmarkConfig()

    names = list(agents.keys())
    assert len(names) >= 2, "Need at least 2 agents to benchmark."

    stats = {n: AgentStats(name=n) for n in names}
    h2h: dict[tuple[str, str], H2HRecord] = {
        (a, b): H2HRecord()
        for a in names
        for b in names
        if a != b
    }
    all_matches: list[MatchResult] = []

    # Build the job list: ordered pairs × games_per_pair
    jobs = []
    for bot_name, player_name in itertools.permutations(names, 2):
        for game_idx in range(config.games_per_pair):
            seed = None
            if config.seed is not None:
                seed = config.seed + hash((bot_name, player_name, game_idx)) % (2**31)
            jobs.append((bot_name, player_name, seed))

    total = len(jobs)
    completed = 0

    def _run_job(job):
        bot_name, player_name, seed = job
        r = play_match(
            bot_policy=agents[bot_name],
            player_policy=agents[player_name],
            bot_name=bot_name,
            player_name=player_name,
            board_n=config.board_n,
            max_plies=config.max_plies,
            seed=seed,
        )
        return r

    if config.n_workers == 1:
        for i, job in enumerate(jobs, 1):
            r = _run_job(job)
            all_matches.append(r)
            _update_stats(stats, r)

            # Update h2h
            bot_n, play_n = r.bot_name, r.player_name
            h2h[(bot_n, play_n)].games += 1
            h2h[(play_n, bot_n)].games += 1
            if r.winner == "bot":
                h2h[(bot_n, play_n)].wins_as_bot += 1
            elif r.winner == "player":
                h2h[(play_n, bot_n)].wins_as_player += 1
            else:
                h2h[(bot_n, play_n)].draws += 1
                h2h[(play_n, bot_n)].draws += 1

            if config.verbose and i % max(1, total // 20) == 0:
                pct = 100 * i / total
                print(f"  [{i:4d}/{total}  {pct:5.1f}%]  last: {bot_n} vs {play_n} → {r.winner}")
    else:
        with ThreadPoolExecutor(max_workers=config.n_workers) as ex:
            futures = {ex.submit(_run_job, j): j for j in jobs}
            for fut in as_completed(futures):
                r = fut.result()
                all_matches.append(r)
                _update_stats(stats, r)
                bot_n, play_n = r.bot_name, r.player_name
                h2h[(bot_n, play_n)].games += 1
                h2h[(play_n, bot_n)].games += 1
                if r.winner == "bot":
                    h2h[(bot_n, play_n)].wins_as_bot += 1
                elif r.winner == "player":
                    h2h[(play_n, bot_n)].wins_as_player += 1
                else:
                    h2h[(bot_n, play_n)].draws += 1
                    h2h[(play_n, bot_n)].draws += 1

                completed += 1
                if config.verbose and completed % max(1, total // 20) == 0:
                    pct = 100 * completed / total
                    print(f"  [{completed:4d}/{total}  {pct:5.1f}%]")

    elo = compute_elo(names, all_matches, k=config.elo_k, base=config.elo_base)
    for name, rating in elo.items():
        stats[name].elo = rating

    return BenchmarkResult(
        config=config,
        agent_names=names,
        all_matches=all_matches,
        stats=stats,
        elo=elo,
        h2h=h2h,
    )
