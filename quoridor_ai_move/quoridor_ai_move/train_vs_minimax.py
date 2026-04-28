# """Train a DQN student against the Python port of the quoridor_move_decision
# minimax engine, with no ROS overhead.

# Same loop as train.py, but the opponent is a fixed alpha-beta search with
# the same evaluation function as the C++/Python minimax engine. Logs all
# the same TensorBoard scalars as train.py plus a student win-rate.

# Run:
#     python -m quoridor_ai_move.train_vs_minimax --episodes 5000 \\
#         --model-dir ./quoridor_models/v_mm \\
#         --tb-log-dir ./quoridor_tensorboard
# """
# from __future__ import annotations

# import argparse
# import json
# import os
# import time
# from collections import deque
# from pathlib import Path

# import numpy as np
# import tensorflow as tf

# try:
#     from .agent import DQNAgent
#     from .encoder import encode_state
#     from .minimax_policy import make_minimax_policy
#     from .replay_buffer import ReplayBuffer
#     from .self_play import play_game
#     from .train import _is_chief, make_strategy
# except ImportError:
#     from agent import DQNAgent
#     from encoder import encode_state
#     from minimax_policy import make_minimax_policy
#     from replay_buffer import ReplayBuffer
#     from self_play import play_game
#     from train import _is_chief, make_strategy


# def main(argv=None):
#     p = argparse.ArgumentParser()
#     p.add_argument("--model-dir", type=str,
#                    default=str(Path.cwd() / "quoridor_models" / "v_mm"))
#     p.add_argument("--resume", action="store_true",
#                    help="Load existing student weights from --model-dir before training")
#     p.add_argument("--minimax-depth", type=int, default=3,
#                    help="Search depth for the minimax teacher (3 matches the C++ default)")
#     p.add_argument("--episodes", type=int, default=2000)
#     p.add_argument("--batch-size", type=int, default=256)
#     p.add_argument("--replay-capacity", type=int, default=200_000)
#     p.add_argument("--target-sync-every", type=int, default=500)
#     p.add_argument("--save-every", type=int, default=100)
#     p.add_argument("--lr", type=float, default=1e-3)
#     p.add_argument("--gamma", type=float, default=0.99)
#     p.add_argument("--tau", type=float, default=0.005,
#                    help="Soft target update rate. Set to 1.0 for hard copy.")
#     p.add_argument("--eps-start", type=float, default=1.0)
#     p.add_argument("--eps-end", type=float, default=0.05)
#     p.add_argument("--eps-decay-episodes", type=int, default=1500)
#     p.add_argument("--student-side", choices=("bot", "player"), default="bot")
#     p.add_argument("--swap-each-episode", action="store_true",
#                    help="Alternate which side the student plays each episode")
#     p.add_argument("--distributed", action="store_true")
#     p.add_argument("--filters", type=int, default=64)
#     p.add_argument("--blocks", type=int, default=4)
#     p.add_argument("--tb-log-dir", type=str, default="./quoridor_tensorboard/")
#     p.add_argument("--shaping-coef", type=float, default=0.1,
#                    help="Potential-based reward shaping strength. 0 disables. "
#                         "Uses path-difference + 0.1*wall-difference (same features as the "
#                         "minimax engine's evaluation). Optimal policy is unchanged "
#                         "(Ng-Harada-Russell), only learning speed.")
#     args = p.parse_args(argv)

#     strategy = make_strategy(args.distributed)

#     student = DQNAgent(lr=args.lr, gamma=args.gamma, tau=args.tau,
#                        strategy=strategy,
#                        filters=args.filters, n_blocks=args.blocks)
#     if args.resume and student.load(args.model_dir):
#         print(f"[train_vs_mm] Resumed student weights from {args.model_dir}")

#     buffer = ReplayBuffer(capacity=args.replay_capacity)
#     minimax_policy = make_minimax_policy(depth=args.minimax_depth)

#     eps_ref = [args.eps_start]

#     def student_policy(board, side, mask):
#         s = encode_state(board, side)
#         return student.select_action(s, mask, eps_ref[0])

#     chief = _is_chief()
#     writer = tf.summary.create_file_writer(args.tb_log_dir) if chief else None
#     if writer:
#         print(f"[train_vs_mm] TensorBoard logs -> {args.tb_log_dir}")
#         print(f"[train_vs_mm] Run: tensorboard --logdir={args.tb_log_dir}")

#     step = 0
#     wins = losses = 0
#     recent_losses: deque[float] = deque(maxlen=50)
#     recent_outcomes: deque[int] = deque(maxlen=50)
#     t0 = time.time()

#     for ep in range(1, args.episodes + 1):
#         eps = max(args.eps_end,
#                   args.eps_start - (args.eps_start - args.eps_end) *
#                   (ep / max(1, args.eps_decay_episodes)))
#         eps_ref[0] = eps

#         # Decide who plays which side this episode.
#         side = args.student_side
#         if args.swap_each_episode and ep % 2 == 0:
#             side = "player" if args.student_side == "bot" else "bot"
#         if side == "bot":
#             bot_pol, player_pol = student_policy, minimax_policy
#             record = ("bot",)
#             want_status = "bot_wins"
#         else:
#             bot_pol, player_pol = minimax_policy, student_policy
#             record = ("player",)
#             want_status = "player_wins"

#         trans, status = play_game(bot_pol, player_pol, record_sides=record,
#                                   shaping_coef=args.shaping_coef, gamma=args.gamma)
#         for t in trans:
#             buffer.add(t.state, t.action, t.reward, t.next_state, t.done, t.next_mask)

#         student_won = (status == want_status)
#         if student_won:
#             wins += 1
#         else:
#             losses += 1
#         recent_outcomes.append(1 if student_won else 0)
#         ep_reward = 1.0 if student_won else (-1.0 if status != "in_progress" else 0.0)
#         ep_length = len(trans)

#         # Train.
#         ep_loss = None
#         if len(buffer) >= args.batch_size:
#             losses_this_ep = []
#             for _ in range(max(1, ep_length)):
#                 batch = buffer.sample(args.batch_size)
#                 losses_this_ep.append(student.train_on_batch(batch))
#                 step += 1
#                 if step % args.target_sync_every == 0:
#                     student.update_target()
#             ep_loss = float(np.mean(losses_this_ep))
#             recent_losses.append(ep_loss)

#         # TensorBoard.
#         if chief and writer is not None:
#             rolling_wr = (sum(recent_outcomes) / len(recent_outcomes)
#                           if recent_outcomes else 0.0)
#             rolling_loss = (sum(recent_losses) / len(recent_losses)
#                             if recent_losses else 0.0)
#             with writer.as_default():
#                 tf.summary.scalar("episode/reward", ep_reward, step=ep)
#                 tf.summary.scalar("episode/length", ep_length, step=ep)
#                 tf.summary.scalar("episode/win", float(student_won), step=ep)
#                 tf.summary.scalar("episode/win_rate_50ep", rolling_wr, step=ep)
#                 if ep_loss is not None:
#                     tf.summary.scalar("episode/loss", ep_loss, step=ep)
#                     tf.summary.scalar("episode/loss_50ep", rolling_loss, step=ep)
#                 tf.summary.scalar("training/epsilon", eps, step=ep)
#                 tf.summary.scalar("training/buffer_size", len(buffer), step=ep)
#                 tf.summary.scalar("training/grad_steps", step, step=ep)

#         if chief and ep % args.save_every == 0:
#             student.save(args.model_dir)
#             dt = time.time() - t0
#             wr = wins / max(1, wins + losses)
#             print(f"[train_vs_mm] ep={ep} eps={eps:.3f} buf={len(buffer)} "
#                   f"student_wr={wr:.2%} steps={step} dt={dt:.1f}s "
#                   f"-> {args.model_dir}")

#     if chief:
#         student.save(args.model_dir)
#         if writer is not None:
#             writer.flush()
#         print(f"[train_vs_mm] Final save to {args.model_dir}")


# if __name__ == "__main__":
#     main()


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
    from quoridor_game.quoridor_utils import Orientation
except ImportError:
    from quoridor_utils import Orientation

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


def _render_board(board, move_desc: str = "") -> str:
    """Return a human-readable ASCII string of the current board state.

    Layout (5x5 example, y increases upward — row y=4 is player's goal,
    row y=0 is bot's goal):

        y=4  .  .  P  .  .      <- player starts here
             |           |
        y=3  .  .  .  .  .
                  ---
        y=2  .  .  .  .  .
        y=1  .  .  .  .  .
        y=0  .  .  B  .  .      <- bot starts here

    Symbols:
        B  bot pawn
        P  player pawn
        .  empty cell
        ---  horizontal wall segment (blocks passage between rows)
        |    vertical wall segment (blocks passage between columns)
    """
    n = board.n_
    # Collect walls by type for fast lookup.
    h_walls: set[tuple[int, int]] = set()   # (wx, wy): blocks between y=wy and y=wy+1
    v_walls: set[tuple[int, int]] = set()   # (wx, wy): blocks between x=wx and x=wx+1
    for w in board.walls:
        if w.orientation == Orientation.HOR:
            h_walls.add(w.pos)
            h_walls.add((w.pos[0] + 1, w.pos[1]))   # walls span 2 cells
        else:
            v_walls.add(w.pos)
            v_walls.add((w.pos[0], w.pos[1] + 1))   # walls span 2 cells

    lines = []
    if move_desc:
        lines.append(f"  Move: {move_desc}")

    # Draw from top (y = n-1) to bottom (y = 0)
    for y in range(n - 1, -1, -1):
        # --- cell row ---
        row_str = f"y={y} "
        for x in range(n):
            if board.bot_pos_.x == x and board.bot_pos_.y == y:
                cell = "B"
            elif board.player_pos_.x == x and board.player_pos_.y == y:
                cell = "P"
            else:
                cell = "."
            row_str += f" {cell}"
            # vertical wall to the right of (x, y)?
            if x < n - 1:
                row_str += "|" if (x, y) in v_walls else " "
        lines.append(row_str)

        # --- horizontal wall row below this cell row ---
        if y > 0:
            wall_str = "    "
            for x in range(n):
                wall_str += "---" if (x, y - 1) in h_walls else "   "
                if x < n - 1:
                    wall_str += " "
            lines.append(wall_str)

    lines.append(
        f"  Bot walls left: {board.bot_walls_remaining}  "
        f"Player walls left: {board.player_walls_remaining}  "
        f"Turn: {board.current_turn}"
    )
    return "\n".join(lines)


def _move_desc(move) -> str:
    """Short human-readable description of a Move object."""
    from quoridor_utils import MoveType
    try:
        from quoridor_game.quoridor_utils import MoveType
    except ImportError:
        pass
    if move is None:
        return "None"
    if move.move_type == MoveType.PAWN:
        return f"pawn -> ({move.target.x}, {move.target.y})"
    o = move.wall.orientation.name
    wx, wy = move.wall.pos
    return f"wall {o} @ ({wx}, {wy})"


def _save_win_game(wins_dir: Path, ep: int, student_side: str,
                   snapshots: list[tuple]) -> None:
    """Write a winning game's board history to wins/<ep>.txt.

    snapshots: list of (board_copy, move, actor_side) in play order.
    """
    wins_dir.mkdir(parents=True, exist_ok=True)
    path = wins_dir / f"ep_{ep:06d}.txt"
    with open(path, "w") as f:
        f.write(f"Episode {ep}  —  student played as: {student_side}\n")
        f.write("=" * 60 + "\n\n")
        for i, (board, move, actor) in enumerate(snapshots):
            desc = f"Ply {i + 1}  [{actor}]  {_move_desc(move)}"
            f.write(_render_board(board, desc) + "\n\n")
        f.write("=" * 60 + "\n")
        f.write(f"Result: {student_side} (student) wins\n")


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
    p.add_argument("--wins-dir", type=str, default="./wins",
                   help="Directory to store board-state logs of games the student wins")
    p.add_argument("--shaping-coef", type=float, default=0.1,
                   help="Potential-based reward shaping strength. 0 disables. "
                        "Uses path-difference + 0.1*wall-difference (same features as the "
                        "minimax engine's evaluation). Optimal policy is unchanged "
                        "(Ng-Harada-Russell), only learning speed.")
    args = p.parse_args(argv)

    strategy = make_strategy(args.distributed)

    student = DQNAgent(lr=args.lr, gamma=args.gamma, tau=args.tau,
                       strategy=strategy,
                       filters=args.filters, n_blocks=args.blocks)
    if args.resume and student.load(args.model_dir):
        print(f"[train_vs_mm] Resumed student weights from {args.model_dir}")

    buffer = ReplayBuffer(capacity=args.replay_capacity)
    minimax_policy = make_minimax_policy(depth=args.minimax_depth)

    eps_ref = [args.eps_start]

    def student_policy(board, side, mask):
        s = encode_state(board, side)
        return student.select_action(s, mask, eps_ref[0])

    chief = _is_chief()
    writer = tf.summary.create_file_writer(args.tb_log_dir) if chief else None
    if writer:
        print(f"[train_vs_mm] TensorBoard logs -> {args.tb_log_dir}")
        print(f"[train_vs_mm] Run: tensorboard --logdir={args.tb_log_dir}")

    wins_dir = Path(args.wins_dir)

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
                _save_win_game(wins_dir, ep, side, snapshots)
        else:
            losses += 1
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
            with writer.as_default():
                tf.summary.scalar("episode/reward", ep_reward, step=ep)
                tf.summary.scalar("episode/length", ep_length, step=ep)
                tf.summary.scalar("episode/win", float(student_won), step=ep)
                tf.summary.scalar("episode/win_rate_50ep", rolling_wr, step=ep)
                if ep_loss is not None:
                    tf.summary.scalar("episode/loss", ep_loss, step=ep)
                    tf.summary.scalar("episode/loss_50ep", rolling_loss, step=ep)
                tf.summary.scalar("training/epsilon", eps, step=ep)
                tf.summary.scalar("training/buffer_size", len(buffer), step=ep)
                tf.summary.scalar("training/grad_steps", step, step=ep)

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


if __name__ == "__main__":
    main()