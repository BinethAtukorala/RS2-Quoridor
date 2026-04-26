"""Train a fresh "student" DQN by playing it against a frozen "teacher"
checkpoint. Useful for iterative improvement: today's best model becomes
tomorrow's teacher.
"""
from __future__ import annotations

import argparse
import os
import time
from pathlib import Path

import tensorflow as tf

from .agent import DQNAgent
from .encoder import encode_state
from .replay_buffer import ReplayBuffer
from .self_play import play_game
from .train import make_strategy, _is_chief


def main(argv=None):
    # Two checkpoints are involved here: --teacher (frozen, opponent) and
    # --model-dir (where the trained student gets saved).
    p = argparse.ArgumentParser()
    p.add_argument("--teacher", type=str, required=True, help="Directory with teacher qnet.weights.h5")
    p.add_argument("--model-dir", type=str, required=True, help="Where to save the new (student) model")
    p.add_argument("--episodes", type=int, default=2000)
    p.add_argument("--batch-size", type=int, default=256)
    p.add_argument("--replay-capacity", type=int, default=200_000)
    p.add_argument("--target-sync-every", type=int, default=500)
    p.add_argument("--save-every", type=int, default=100)
    p.add_argument("--lr", type=float, default=1e-3)
    p.add_argument("--gamma", type=float, default=0.99)
    p.add_argument("--eps-start", type=float, default=1.0)
    p.add_argument("--eps-end", type=float, default=0.05)
    p.add_argument("--eps-decay-episodes", type=int, default=1500)
    p.add_argument("--teacher-eps", type=float, default=0.05,
                   help="Small exploration for the teacher so games aren't deterministic")
    p.add_argument("--student-side", choices=("bot", "player", "both"), default="both",
                   help="Which side(s) the student plays; the teacher plays the other")
    p.add_argument("--swap-each-episode", action="store_true",
                   help="Alternate which side the student plays each episode (only with --student-side bot or player)")
    p.add_argument("--distributed", action="store_true")
    p.add_argument("--filters", type=int, default=64)
    p.add_argument("--blocks", type=int, default=4)
    args = p.parse_args(argv)

    strategy = make_strategy(args.distributed)

    # Teacher: frozen, inference-only. Its weights never change during training.
    teacher = DQNAgent(lr=1e-4, gamma=args.gamma, strategy=strategy,
                       filters=args.filters, n_blocks=args.blocks)
    if not teacher.load(args.teacher):
        raise FileNotFoundError(f"No teacher weights at {args.teacher}")
    print(f"[train_vs] Loaded teacher from {args.teacher}")

    # Student: trainable. Starts from a fresh random init by default.
    student = DQNAgent(lr=args.lr, gamma=args.gamma, strategy=strategy,
                       filters=args.filters, n_blocks=args.blocks)

    buffer = ReplayBuffer(capacity=args.replay_capacity)

    # Closures wrap the two agents into the (board, side, mask) -> action
    # signature that play_game expects.
    def student_policy_factory(eps_ref):
        def policy(board, side, mask):
            s = encode_state(board, side)
            return student.select_action(s, mask, eps_ref[0])
        return policy

    def teacher_policy(board, side, mask):
        # Teacher uses a small fixed epsilon so its games aren't perfectly
        # deterministic (otherwise the student would see zero variety).
        s = encode_state(board, side)
        return teacher.select_action(s, mask, args.teacher_eps)

    eps_ref = [args.eps_start]
    student_policy = student_policy_factory(eps_ref)

    chief = _is_chief()
    step = 0
    wins = losses = 0
    t0 = time.time()

    for ep in range(1, args.episodes + 1):
        # Linearly anneal exploration just like in train.py.
        eps = max(args.eps_end,
                  args.eps_start - (args.eps_start - args.eps_end) *
                  (ep / max(1, args.eps_decay_episodes)))
        eps_ref[0] = eps

        # Decide who plays which side this episode and which side(s) we
        # actually record transitions for (only the student's side).
        if args.student_side == "both":
            bot_pol, player_pol = student_policy, student_policy
            record = ("bot", "player")
        else:
            side = args.student_side
            if args.swap_each_episode and ep % 2 == 0:
                # Swap so the student gets practice on both colors.
                side = "player" if args.student_side == "bot" else "bot"
            if side == "bot":
                bot_pol, player_pol = student_policy, teacher_policy
                record = ("bot",)
            else:
                bot_pol, player_pol = teacher_policy, student_policy
                record = ("player",)

        trans, status = play_game(bot_pol, player_pol, record_sides=record)
        for t in trans:
            buffer.add(t.state, t.action, t.reward, t.next_state, t.done, t.next_mask)

        # Track student win-rate for logging (used to monitor improvement).
        student_won = False
        if args.student_side == "both":
            student_won = status in ("bot_wins", "player_wins")  # always
        else:
            want = "bot_wins" if ("bot" in record) else "player_wins"
            student_won = status == want
        if student_won:
            wins += 1
        else:
            losses += 1

        # One gradient step per game ply, with periodic target-network sync.
        if len(buffer) >= args.batch_size:
            for _ in range(max(1, len(trans))):
                batch = buffer.sample(args.batch_size)
                student.train_on_batch(batch)
                step += 1
                if step % args.target_sync_every == 0:
                    student.update_target()

        if chief and ep % args.save_every == 0:
            student.save(args.model_dir)
            dt = time.time() - t0
            wr = wins / max(1, wins + losses)
            print(f"[train_vs] ep={ep} eps={eps:.3f} buf={len(buffer)} "
                  f"student_winrate={wr:.2%} steps={step} dt={dt:.1f}s "
                  f"-> saved to {args.model_dir}")

    if chief:
        student.save(args.model_dir)
        print(f"[train_vs] Final save to {args.model_dir}")


if __name__ == "__main__":
    main()
