"""Train a DQN student against the Python port of the quoridor_move_decision
minimax engine, with no ROS overhead.

Same loop as train.py, but the opponent is a fixed alpha-beta search with
the same evaluation function as the C++/Python minimax engine. Logs all
the same TensorBoard scalars as train.py plus a student win-rate.

Run:
    python -m quoridor_ai_move.train_vs_minimax --episodes 5000 \\
        --model-dir ./quoridor_models/v_mm \\
        --tb-log-dir ./quoridor_tensorboard
"""
from __future__ import annotations

import argparse
import json
import os
import time
from collections import deque
from pathlib import Path

import numpy as np
import tensorflow as tf

try:
    from .agent import DQNAgent
    from .encoder import encode_state
    from .minimax_policy import make_minimax_policy
    from .replay_buffer import ReplayBuffer
    from .self_play import play_game
    from .train import _is_chief, make_strategy
except ImportError:
    from agent import DQNAgent
    from encoder import encode_state
    from minimax_policy import make_minimax_policy
    from replay_buffer import ReplayBuffer
    from self_play import play_game
    from train import _is_chief, make_strategy


def _move_desc(move) -> str:
    """Short human-readable description of a Move object."""
    try:
        from quoridor_game.quoridor_utils import MoveType
    except ImportError:
        from quoridor_utils import MoveType
    if move is None:
        return "None"
    if move.move_type == MoveType.PAWN:
        return f"pawn -> ({move.target.x}, {move.target.y})"
    o = move.wall.orientation.name
    wx, wy = move.wall.pos
    return f"wall {o} @ ({wx}, {wy})"


def _save_game(output_dir: Path, ep: int, student_side: str,
               snapshots: list[tuple], won: bool) -> None:
    """Write a game's board history to output_dir/ep_<N>.txt.

    snapshots: list of (board_copy, move, actor_side) in play order.
    won: True if the student won, False if they lost.
    """
    output_dir.mkdir(parents=True, exist_ok=True)
    path = output_dir / f"ep_{ep:06d}.txt"
    result_str = "wins" if won else "loses"
    with open(path, "w") as f:
        f.write(f"Episode {ep}  —  student played as: {student_side}\n")
        f.write("=" * 60 + "\n\n")
        for i, (board, move, actor) in enumerate(snapshots):
            f.write(f"Ply {i + 1}  [{actor}]  {_move_desc(move)}\n")
            f.write(board.display() + "\n\n")
        f.write("=" * 60 + "\n")
        f.write(f"Result: {student_side} (student) {result_str}\n")


def _game_fingerprint(snapshots: list[tuple]) -> tuple[str, ...]:
    """Return a tuple of move-description strings — one per ply.
    Two games with the same fingerprint played exactly the same moves."""
    return tuple(_move_desc(move) for _, move, _ in snapshots)


def _write_similar_report(parent_dir: Path, label: str,
                           ep_fingerprints: dict[int, tuple[str, ...]]) -> None:
    """Scan ep_fingerprints for duplicate move sequences and write a report.

    Creates parent_dir/similar/<label>.txt  (e.g. wins.txt / losses.txt).
    Format:
        same game 1 - 3, 27, 86
        same game 2 - 14, 25, 57
    Only groups with 2+ episodes are listed.
    """
    if not ep_fingerprints:
        return

    # Invert: fingerprint -> list of episode numbers
    from collections import defaultdict
    groups: dict[tuple, list[int]] = defaultdict(list)
    for ep, fp in ep_fingerprints.items():
        groups[fp].append(ep)

    duplicates = sorted(
        [sorted(eps) for eps in groups.values() if len(eps) > 1],
        key=lambda g: g[0],
    )
    if not duplicates:
        return

    similar_dir = parent_dir / "similar"
    similar_dir.mkdir(parents=True, exist_ok=True)
    path = similar_dir / f"{label}.txt"
    with open(path, "w") as f:
        f.write(f"Repeated games in {label}\n")
        f.write("=" * 40 + "\n\n")
        for i, group in enumerate(duplicates, 1):
            eps_str = ", ".join(str(e) for e in group)
            f.write(f"same game {i} - {eps_str}\n")
    print(f"[train_vs_mm] Similar-game report -> {path}  "
          f"({len(duplicates)} repeated sequence(s) found)")


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("--model-dir", type=str,
                   default=str(Path.cwd() / "quoridor_models" / "v_mm"))
    p.add_argument("--resume", action="store_true",
                   help="Load existing student weights from --model-dir before training")
    p.add_argument("--minimax-depth", type=int, default=3,
                   help="Search depth for the minimax teacher (3 matches the C++ default)")
    p.add_argument("--episodes", type=int, default=2000)
    p.add_argument("--batch-size", type=int, default=256)
    p.add_argument("--replay-capacity", type=int, default=200_000)
    p.add_argument("--target-sync-every", type=int, default=500)
    p.add_argument("--save-every", type=int, default=100)
    p.add_argument("--lr", type=float, default=1e-3)
    p.add_argument("--gamma", type=float, default=0.99)
    p.add_argument("--tau", type=float, default=0.005,
                   help="Soft target update rate. Set to 1.0 for hard copy.")
    p.add_argument("--eps-start", type=float, default=1.0)
    p.add_argument("--eps-end", type=float, default=0.05)
    p.add_argument("--eps-decay-episodes", type=int, default=1500)
    p.add_argument("--student-side", choices=("bot", "player"), default="bot")
    p.add_argument("--swap-each-episode", action="store_true",
                   help="Alternate which side the student plays each episode")
    p.add_argument("--distributed", action="store_true")
    p.add_argument("--filters", type=int, default=64)
    p.add_argument("--blocks", type=int, default=4)
    p.add_argument("--tb-log-dir", type=str, default="./quoridor_tensorboard/")
    p.add_argument("--games-dir", type=str, default="./games",
                   help="Directory to store board-state logs of all games (wins/ and losses/ subfolders)")
    p.add_argument("--shaping-coef", type=float, default=0.1,
                   help="Potential-based reward shaping strength. 0 disables. "
                        "Uses path-difference + 0.1*wall-difference (same features as the "
                        "minimax engine's evaluation). Optimal policy is unchanged "
                        "(Ng-Harada-Russell), only learning speed.")
    p.add_argument("--opponent-noise", type=float, default=0.1,
                   help="Probability that the minimax opponent plays a uniformly random "
                        "legal move instead of its best move. Prevents game repetition "
                        "when the student's epsilon is low. 0 = fully deterministic "
                        "minimax (original behaviour). Default 0.1.")
    args = p.parse_args(argv)

    strategy = make_strategy(args.distributed)

    student = DQNAgent(lr=args.lr, gamma=args.gamma, tau=args.tau,
                       strategy=strategy,
                       filters=args.filters, n_blocks=args.blocks)
    if args.resume and student.load(args.model_dir):
        print(f"[train_vs_mm] Resumed student weights from {args.model_dir}")

    buffer = ReplayBuffer(capacity=args.replay_capacity)
    minimax_policy = make_minimax_policy(depth=args.minimax_depth,
                                         opponent_noise=args.opponent_noise)

    eps_ref = [args.eps_start]

    def student_policy(board, side, mask):
        s = encode_state(board, side)
        return student.select_action(s, mask, eps_ref[0])

    chief = _is_chief()
    writer = tf.summary.create_file_writer(args.tb_log_dir) if chief else None
    if writer:
        print(f"[train_vs_mm] TensorBoard logs -> {args.tb_log_dir}")
        print(f"[train_vs_mm] Run: tensorboard --logdir={args.tb_log_dir}")

    games_dir = Path(args.games_dir)
    wins_dir   = games_dir / "wins"
    losses_dir = games_dir / "losses"

    # Track move fingerprints so we can report repeated games at the end.
    win_fingerprints:  dict[int, tuple[str, ...]] = {}
    loss_fingerprints: dict[int, tuple[str, ...]] = {}

    step = 0
    wins = losses = 0
    recent_losses: deque[float] = deque(maxlen=50)
    recent_outcomes: deque[int] = deque(maxlen=50)
    t0 = time.time()

    for ep in range(1, args.episodes + 1):
        eps = max(args.eps_end,
                  args.eps_start - (args.eps_start - args.eps_end) *
                  (ep / max(1, args.eps_decay_episodes)))
        eps_ref[0] = eps

        # Decide who plays which side this episode.
        side = args.student_side
        if args.swap_each_episode and ep % 2 == 0:
            side = "player" if args.student_side == "bot" else "bot"
        if side == "bot":
            bot_pol, player_pol = student_policy, minimax_policy
            record = ("bot",)
            want_status = "bot_wins"
        else:
            bot_pol, player_pol = minimax_policy, student_policy
            record = ("player",)
            want_status = "player_wins"

        # Wrap both policies to record (board_snapshot, move, actor) per ply.
        snapshots: list[tuple] = []

        def _wrap(policy, actor_side):
            def wrapped(board, side, mask):
                action = policy(board, side, mask)
                # Snapshot the board state *before* the move is applied,
                # paired with the chosen move (decoded for display).
                try:
                    from quoridor_ai_move.encoder import action_to_move
                except ImportError:
                    from encoder import action_to_move
                move = action_to_move(board, action, side)
                snapshots.append((board.copy(), move, actor_side))
                return action
            return wrapped

        if side == "bot":
            bot_pol_ep  = _wrap(student_policy, "bot (student)")
            player_pol_ep = _wrap(minimax_policy, "player (minimax)")
        else:
            bot_pol_ep  = _wrap(minimax_policy, "bot (minimax)")
            player_pol_ep = _wrap(student_policy, "player (student)")

        trans, status = play_game(bot_pol_ep, player_pol_ep, record_sides=record,
                                  shaping_coef=args.shaping_coef, gamma=args.gamma)
        for t in trans:
            buffer.add(t.state, t.action, t.reward, t.next_state, t.done, t.next_mask)

        student_won = (status == want_status)
        if student_won:
            wins += 1
            if chief:
                _save_game(wins_dir, ep, side, snapshots, won=True)
                win_fingerprints[ep] = _game_fingerprint(snapshots)
        else:
            losses += 1
            if chief:
                _save_game(losses_dir, ep, side, snapshots, won=False)
                loss_fingerprints[ep] = _game_fingerprint(snapshots)
        recent_outcomes.append(1 if student_won else 0)
        ep_reward = 1.0 if student_won else (-1.0 if status != "in_progress" else 0.0)
        ep_length = len(trans)

        # Train.
        ep_loss = None
        if len(buffer) >= args.batch_size:
            losses_this_ep = []
            for _ in range(max(1, ep_length)):
                batch = buffer.sample(args.batch_size)
                losses_this_ep.append(student.train_on_batch(batch))
                step += 1
                if step % args.target_sync_every == 0:
                    student.update_target()
            ep_loss = float(np.mean(losses_this_ep))
            recent_losses.append(ep_loss)

        # TensorBoard.
        if chief and writer is not None:
            rolling_wr = (sum(recent_outcomes) / len(recent_outcomes)
                          if recent_outcomes else 0.0)
            rolling_loss = (sum(recent_losses) / len(recent_losses)
                            if recent_losses else 0.0)

            # Track game length with a rolling average (less noisy than per-ep).
            if not hasattr(main, '_recent_lengths'):
                main._recent_lengths = deque(maxlen=50)
            main._recent_lengths.append(ep_length)
            avg_length_50 = float(np.mean(main._recent_lengths))

            # Decisive = actually ended in win/loss, not a timeout.
            timed_out = (status == "in_progress")
            decisive_recent = sum(
                1 for o in recent_outcomes if o != 0
            )
            decisive_rate = (decisive_recent / len(recent_outcomes)
                             if recent_outcomes else 0.0)

            with writer.as_default():
                tf.summary.scalar("episode/reward",        ep_reward,    step=ep)
                tf.summary.scalar("episode/length",        ep_length,    step=ep)
                tf.summary.scalar("episode/length_avg50",  avg_length_50, step=ep)
                tf.summary.scalar("episode/win",           float(student_won), step=ep)
                tf.summary.scalar("episode/win_rate_50ep", rolling_wr,   step=ep)
                tf.summary.scalar("episode/decisive_rate_50", decisive_rate, step=ep)
                tf.summary.scalar("episode/timed_out",     float(timed_out), step=ep)
                tf.summary.scalar("episode/cumulative_wr",
                                  wins / max(1, wins + losses),          step=ep)
                if ep_loss is not None:
                    tf.summary.scalar("episode/loss",      ep_loss,      step=ep)
                    tf.summary.scalar("episode/loss_50ep", rolling_loss, step=ep)
                tf.summary.scalar("training/epsilon",      eps,          step=ep)
                tf.summary.scalar("training/buffer_size",  len(buffer),  step=ep)
                tf.summary.scalar("training/grad_steps",   step,         step=ep)

        if chief and ep % args.save_every == 0:
            student.save(args.model_dir)
            dt = time.time() - t0
            wr = wins / max(1, wins + losses)
            print(f"[train_vs_mm] ep={ep} eps={eps:.3f} buf={len(buffer)} "
                  f"student_wr={wr:.2%} steps={step} dt={dt:.1f}s "
                  f"-> {args.model_dir}")

    if chief:
        student.save(args.model_dir)
        if writer is not None:
            writer.flush()
        print(f"[train_vs_mm] Final save to {args.model_dir}")
        _write_similar_report(games_dir, "wins",   win_fingerprints)
        _write_similar_report(games_dir, "losses", loss_fingerprints)


if __name__ == "__main__":
    main()