"""Benchmark report generation: console table + JSON/CSV export."""
from __future__ import annotations

import json
import math
from pathlib import Path
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .benchmark import BenchmarkResult


# ---------------------------------------------------------------------------
# Main report
# ---------------------------------------------------------------------------

def generate_report(
    result: "BenchmarkResult",
    output_dir: str | None = None,
    print_report: bool = True,
) -> str:
    """Render a full benchmark report.

    Parameters
    ----------
    result : BenchmarkResult
    output_dir : str | None
        If given, writes report.txt, results.json, and h2h.csv here.
    print_report : bool
        Print to stdout.

    Returns
    -------
    str : The rendered report text.
    """
    lines: list[str] = []
    cfg = result.config

    def ln(s: str = ""):
        lines.append(s)

    # ── Header ────────────────────────────────────────────────────────────
    ln("  QUORIDOR AI BENCHMARK REPORT")
    ln(f"  Board: {cfg.board_n}x{cfg.board_n}   "
       f"Games/pair: {cfg.games_per_pair}   "
       f"Total games: {len(result.all_matches)}")
    ln()

    # Sort ranking by win rate descending
    ranking_by_winrate = sorted(
        result.ranking,
        key=lambda x: result.stats[x[0]].win_rate,
        reverse=True,
    )

    # ── Rankings ──────────────────────────────────────────────────────────
    ln("  OVERALL RANKING  (by Win Rate)")
    ln("  " + "-" * 66)
    ln(f"  {'Rank':<5} {'Agent':<28} {'Win%':>6} {'W':>6} {'L':>6} {'D':>6}")
    ln("  " + "-" * 66)

    for rank, (name, _elo) in enumerate(ranking_by_winrate, 1):
        s = result.stats[name]
        ln(
            f"  {rank:<5} {name:<28} {s.win_rate:>5.1%} {s.wins:>6} {s.losses:>6} {s.draws:>6}"
        )
    ln()

    # ── Per-agent stats ────────────────────────────────────────────────────
    ln("  PER-AGENT STATS")
    for name, _ in ranking_by_winrate:
        s = result.stats[name]
        ln(f"  {name}")
        ln(f"    Win rate : {s.win_rate:.1%}")
        ln(f"    Avg plies: {s.avg_plies:.1f}")
        ln(f"    Move time: avg {s.avg_move_ms:.1f} ms   max {s.max_move_ms:.0f} ms")
        ln()

    # ── Head-to-head ──────────────────────────────────────────────────────
    names = result.agent_names
    col_w = 12
    table_width = 24 + col_w * len(names)
    ln("  HEAD-TO-HEAD WIN RATES")
    ln("  " + "-" * table_width)
    ln("  " + " " * 22 + "".join(f"{n[:10]:>{col_w}}" for n in names))
    for a in names:
        row = f"  {a:<22}"
        for b in names:
            if a == b:
                row += f"{'—':>{col_w}}"
            else:
                rec = result.h2h.get((a, b))
                if rec and rec.games:
                    wr = rec.total_wins / rec.games
                    row += f"{wr:>{col_w-1}.1%} "
                else:
                    row += f"{'N/A':>{col_w}}"
        ln(row)
    ln()

    # ── Longest / Shortest games ──────────────────────────────────────────
    all_m = result.all_matches
    if all_m:
        sorted_by_plies = sorted(all_m, key=lambda r: r.plies)
        shortest = sorted_by_plies[0]
        longest  = sorted_by_plies[-1]
        ln("  GAME EXTREMES")
        ln(f"    Shortest: {shortest.plies} plies  "
           f"({shortest.bot_name} vs {shortest.player_name} -> {shortest.winner} wins)")
        ln(f"    Longest : {longest.plies} plies  "
           f"({longest.bot_name} vs {longest.player_name} -> {longest.winner} wins)")
        draws = sum(1 for r in all_m if r.winner == "draw")
        ln(f"    Draws/timeouts: {draws}  ({100*draws/len(all_m):.1f}%)")
    ln()

    report_text = "\n".join(lines)

    if print_report:
        print(report_text)

    # ── File export ────────────────────────────────────────────────────────
    if output_dir:
        out = Path(output_dir)
        out.mkdir(parents=True, exist_ok=True)

        # report.txt
        (out / "report.txt").write_text(report_text)

        # results.json
        json_data = {
            "config": {
                "games_per_pair": cfg.games_per_pair,
                "board_n": cfg.board_n,
                "max_plies": cfg.max_plies,
                "seed": cfg.seed,
                "elo_k": cfg.elo_k,
                "elo_base": cfg.elo_base,
            },
            "ranking": [
                {
                    "rank": i + 1,
                    "name": name,
                    "win_rate": round(result.stats[name].win_rate, 4),
                    "wins": result.stats[name].wins,
                    "losses": result.stats[name].losses,
                    "draws": result.stats[name].draws,
                    "avg_plies": round(result.stats[name].avg_plies, 1),
                    "avg_move_ms": round(result.stats[name].avg_move_ms, 2),
                    "max_move_ms": round(result.stats[name].max_move_ms, 2),
                }
                for i, (name, _elo) in enumerate(ranking_by_winrate)
            ],
            "head_to_head": [
                {
                    "agent_a": a,
                    "agent_b": b,
                    "games": rec.games,
                    "wins_a": rec.total_wins,
                    "win_rate_a": round(rec.win_rate, 4),
                }
                for (a, b), rec in result.h2h.items()
                if rec.games
            ],
            "total_games": len(result.all_matches),
        }
        (out / "results.json").write_text(json.dumps(json_data, indent=2))

        # h2h.csv
        csv_lines = ["agent_a,agent_b,games,wins_a,losses_a,draws,win_rate_a"]
        for a in names:
            for b in names:
                if a == b:
                    continue
                rec = result.h2h.get((a, b))
                if rec and rec.games:
                    losses_a = rec.games - rec.total_wins - rec.draws
                    csv_lines.append(
                        f"{a},{b},{rec.games},{rec.total_wins},"
                        f"{losses_a},{rec.draws},{rec.win_rate:.4f}"
                    )
        (out / "h2h.csv").write_text("\n".join(csv_lines))

        # match_log.csv — one row per game
        log_lines = ["game_num,bot,player,winner,plies,status,bot_avg_ms,player_avg_ms"]
        for i, r in enumerate(result.all_matches, 1):
            bot_times  = r.move_times_ms.get("bot", [])
            play_times = r.move_times_ms.get("player", [])
            bot_ms  = sum(bot_times)  / max(1, len(bot_times))
            play_ms = sum(play_times) / max(1, len(play_times))
            log_lines.append(
                f"{i},{r.bot_name},{r.player_name},{r.winner},"
                f"{r.plies},{r.status},{bot_ms:.1f},{play_ms:.1f}"
            )
        (out / "match_log.csv").write_text("\n".join(log_lines))

        print(f"\n  -> Saved to: {out.resolve()}/")
        print(f"     report.txt | results.json | h2h.csv | match_log.csv")

    return report_text
