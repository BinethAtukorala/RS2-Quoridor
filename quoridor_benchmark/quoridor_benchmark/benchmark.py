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
    """Total number of games between each unordered pair (A, B).
    Half are played with A as bot, half with B as bot.
    e.g. games_per_pair=200 → 100 games each direction → 200 total."""

    matchups: list[tuple[str, str]] | None = None
    """Optional explicit list of unordered pairs to test.
    If None, all pairs are tested (full round-robin).
    e.g. [("AlphaZero", "DQN-1"), ("AlphaZero", "DQN-2")]
    will only run AlphaZero vs each DQN, skipping DQN vs DQN."""

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
    """Converged Elo via multiple shuffled passes with decaying K."""
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
        """Wins / (wins + losses). Draws excluded from denominator."""
        decisive = self.wins + self.losses
        return self.wins / decisive if decisive else 0.0

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
        bot_s.wins    += 1
        play_s.losses += 1
    elif r.winner == "player":
        play_s.wins  += 1
        bot_s.losses += 1
    else:
        bot_s.draws  += 1
        play_s.draws += 1


# ---------------------------------------------------------------------------
# Head-to-head record  (symmetric, unordered pairs)
# ---------------------------------------------------------------------------

@dataclass
class H2HRecord:
    """Symmetric head-to-head record between an unordered pair of agents."""
    wins: int = 0
    losses: int = 0
    draws: int = 0
    games: int = 0

    @property
    def win_rate(self) -> float:
        """Win rate excluding draws from denominator."""
        decisive = self.wins + self.losses
        return self.wins / decisive if decisive else 0.0


def _canonical(a: str, b: str) -> tuple[str, str]:
    """Return a consistent ordering for an unordered pair."""
    return (a, b) if a < b else (b, a)


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
    h2h: dict[tuple[str, str], H2HRecord]
    """Keys are canonical (alphabetically sorted) pairs.
    h2h[(a, b)].wins = number of times *a* beat *b* (regardless of role)."""

    def h2h_win_rate(self, agent: str, opponent: str) -> float:
        """Win rate of *agent* against *opponent* (draws excluded from denominator)."""
        key = _canonical(agent, opponent)
        rec = self.h2h.get(key)
        if rec is None or rec.games == 0:
            return 0.0
        wins   = rec.wins   if key[0] == agent else rec.losses
        losses = rec.losses if key[0] == agent else rec.wins
        decisive = wins + losses
        return wins / decisive if decisive else 0.0

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
    """Run a benchmark tournament.

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

    # Resolve which unordered pairs to run
    if config.matchups is not None:
        pairs = [_canonical(a, b) for a, b in config.matchups]
        for a, b in pairs:
            assert a in agents, f"Agent '{a}' in matchups not found in agents dict."
            assert b in agents, f"Agent '{b}' in matchups not found in agents dict."
        # Deduplicate in case user passed the same pair twice
        pairs = list(dict.fromkeys(pairs))
    else:
        pairs = [_canonical(a, b) for a in names for b in names if a < b]

    stats = {n: AgentStats(name=n) for n in names}
    h2h: dict[tuple[str, str], H2HRecord] = {pair: H2HRecord() for pair in pairs}
    all_matches: list[MatchResult] = []

    # games_per_pair is total; split evenly each direction
    games_each_direction = config.games_per_pair // 2

    # Build job list: both directions per pair × games_each_direction
    jobs = []
    for a, b in pairs:
        for game_idx in range(games_each_direction):
            for bot_name, player_name in [(a, b), (b, a)]:
                seed = None
                if config.seed is not None:
                    seed = config.seed + hash((bot_name, player_name, game_idx)) % (2**31)
                jobs.append((bot_name, player_name, seed))

    total = len(jobs)
    completed = 0

    def _run_job(job):
        bot_name, player_name, seed = job
        return play_match(
            bot_policy=agents[bot_name],
            player_policy=agents[player_name],
            bot_name=bot_name,
            player_name=player_name,
            board_n=config.board_n,
            max_plies=config.max_plies,
            seed=seed,
        )

    def _record_match(r: MatchResult):
        _update_stats(stats, r)

        key = _canonical(r.bot_name, r.player_name)
        rec = h2h[key]
        rec.games += 1

        if r.winner == "draw":
            rec.draws += 1
        else:
            winner_name = r.bot_name if r.winner == "bot" else r.player_name
            if key[0] == winner_name:
                rec.wins += 1
            else:
                rec.losses += 1

    if config.n_workers == 1:
        for i, job in enumerate(jobs, 1):
            r = _run_job(job)
            all_matches.append(r)
            _record_match(r)

            if config.verbose and i % max(1, total // 20) == 0:
                pct = 100 * i / total
                bot_n, play_n = job[0], job[1]
                print(f"  [{i:4d}/{total}  {pct:5.1f}%]  last: {bot_n} vs {play_n} → {r.winner}")
    else:
        with ThreadPoolExecutor(max_workers=config.n_workers) as ex:
            futures = {ex.submit(_run_job, j): j for j in jobs}
            for fut in as_completed(futures):
                r = fut.result()
                all_matches.append(r)
                _record_match(r)

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