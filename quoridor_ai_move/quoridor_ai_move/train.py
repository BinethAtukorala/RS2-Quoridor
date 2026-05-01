"""Offline self-play training entry point.

Repeatedly plays full games where both sides use the same (improving)
policy, stores the transitions in a replay buffer, and runs DQN updates.
Run as: `python -m quoridor_ai_move.train --episodes ...`.
"""
from __future__ import annotations

import argparse
import json
import os
import time
from pathlib import Path
from collections import defaultdict, deque  # added deque

import numpy as np
import tensorflow as tf

try:
    from .agent import DQNAgent
    from .replay_buffer import ReplayBuffer
    from .self_play import play_game
except ImportError:
    from agent import DQNAgent
    from replay_buffer import ReplayBuffer
    from self_play import play_game


# =========================
# ADDED: Logging helpers
# =========================

def _move_desc(move) -> str:
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


def _save_game(output_dir: Path, ep: int, snapshots: list[tuple], result: str) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    path = output_dir / f"ep_{ep:06d}.txt"

    with open(path, "w") as f:
        f.write(f"Episode {ep}\n")
        f.write("=" * 60 + "\n\n")

        for i, (board, move, actor) in enumerate(snapshots):
            f.write(f"Ply {i+1} [{actor}] {_move_desc(move)}\n")
            f.write(board.display() + "\n\n")

        f.write("=" * 60 + "\n")
        f.write(f"Result: {result}\n")


def _game_fingerprint(snapshots: list[tuple]) -> tuple[str, ...]:
    return tuple(_move_desc(move) for _, move, _ in snapshots)


def _write_similar_report(parent_dir: Path, label: str,
                         ep_fingerprints: dict[int, tuple[str, ...]]) -> None:
    if not ep_fingerprints:
        return

    groups = defaultdict(list)
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
            f.write(f"same game {i} - {', '.join(map(str, group))}\n")

    print(f"[train] Similar-game report -> {path}")


# =========================


def make_strategy(distributed: bool) -> tf.distribute.Strategy:
    gpus = tf.config.list_physical_devices("GPU")
    for g in gpus:
        try:
            tf.config.experimental.set_memory_growth(g, True)
        except Exception:
            pass
    if distributed:
        if "TF_CONFIG" not in os.environ:
            raise RuntimeError("Set TF_CONFIG env var to use --distributed.")
        return tf.distribute.MultiWorkerMirroredStrategy()
    if len(gpus) > 1:
        return tf.distribute.MirroredStrategy()
    return tf.distribute.get_strategy()


def _epsilon_greedy_factory(agent: DQNAgent, eps_ref: list[float]):
    def policy(board, side, mask):
        try:
            from .encoder import encode_state
        except ImportError:
            from encoder import encode_state

        s = encode_state(board, side)
        return agent.select_action(s, mask, eps_ref[0])
    return policy


def _is_chief() -> bool:
    tfc = os.environ.get("TF_CONFIG")
    if not tfc:
        return True
    try:
        d = json.loads(tfc)
        return d.get("task", {}).get("index", 0) == 0
    except Exception:
        return True


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("--episodes", type=int, default=2000)
    p.add_argument("--model-dir", type=str, default=str(Path.cwd() / "quoridor_models" / "latest"))
    p.add_argument("--resume", action="store_true")
    p.add_argument("--batch-size", type=int, default=256)
    p.add_argument("--replay-capacity", type=int, default=200_000)
    p.add_argument("--train-every", type=int, default=1)
    p.add_argument("--target-sync-every", type=int, default=500)
    p.add_argument("--save-every", type=int, default=100)
    p.add_argument("--lr", type=float, default=1e-3)
    p.add_argument("--gamma", type=float, default=0.99)
    p.add_argument("--eps-start", type=float, default=1.0)
    p.add_argument("--eps-end", type=float, default=0.05)
    p.add_argument("--eps-decay-episodes", type=int, default=1500)
    p.add_argument("--distributed", action="store_true")
    p.add_argument("--filters", type=int, default=64)
    p.add_argument("--blocks", type=int, default=4)
    p.add_argument("--tau", type=float, default=0.005)
    p.add_argument("--tb-log-dir", type=str, default="./quoridor_tensorboard/")
    p.add_argument("--games-dir", type=str, default="./games")

    args = p.parse_args(argv)

    strategy = make_strategy(args.distributed)
    agent = DQNAgent(lr=args.lr, gamma=args.gamma, strategy=strategy,
                     filters=args.filters, n_blocks=args.blocks)

    if args.resume and agent.load(args.model_dir):
        print(f"[train] Resumed weights from {args.model_dir}")

    buffer = ReplayBuffer(capacity=args.replay_capacity)
    eps_ref = [args.eps_start]
    policy = _epsilon_greedy_factory(agent, eps_ref)

    chief = _is_chief()
    step = 0
    wins = losses = draws = 0
    t0 = time.time()

    writer = tf.summary.create_file_writer(args.tb_log_dir) if chief else None

    # ADDED PRINTS
    if writer:
        print(f"[train_vs_mm] TensorBoard logs -> {args.tb_log_dir}")
        print(f"[train_vs_mm] Run: tensorboard --logdir={args.tb_log_dir}")

    recent_losses = deque(maxlen=50)        #  changed
    recent_outcomes = deque(maxlen=50)      #  added

    games_dir = Path(args.games_dir)
    wins_dir = games_dir / "wins"
    losses_dir = games_dir / "losses"
    draws_dir = games_dir / "draws"

    win_fp = {}
    loss_fp = {}
    draw_fp = {}

    for ep in range(1, args.episodes + 1):

        eps = max(args.eps_end,
                  args.eps_start - (args.eps_start - args.eps_end) *
                  (ep / max(1, args.eps_decay_episodes)))
        eps_ref[0] = eps

        snapshots = []

        def _wrap(policy, label):
            def wrapped(board, side, mask):
                action = policy(board, side, mask)
                try:
                    from .encoder import action_to_move
                except ImportError:
                    from encoder import action_to_move

                move = action_to_move(board, action, side)
                snapshots.append((board.copy(), move, label))
                return action
            return wrapped

        bot_pol = _wrap(policy, "bot")
        player_pol = _wrap(policy, "player")

        trans, status = play_game(bot_pol, player_pol, record_sides=("bot", "player"))

        for t in trans:
            buffer.add(t.state, t.action, t.reward, t.next_state, t.done, t.next_mask)

        if status == "bot_wins":
            wins += 1
            if chief:
                _save_game(wins_dir, ep, snapshots, "bot wins")
                win_fp[ep] = _game_fingerprint(snapshots)
            recent_outcomes.append(1)

        elif status == "player_wins":
            losses += 1
            if chief:
                _save_game(losses_dir, ep, snapshots, "player wins")
                loss_fp[ep] = _game_fingerprint(snapshots)
            recent_outcomes.append(0)

        else:
            draws += 1
            if chief:
                _save_game(draws_dir, ep, snapshots, "draw")
                draw_fp[ep] = _game_fingerprint(snapshots)
            recent_outcomes.append(0)

        ep_reward = 1.0 if status == "bot_wins" else (-1.0 if status == "player_wins" else 0.0)
        ep_length = len(trans)
        ep_loss = None

        if len(buffer) >= args.batch_size:
            batch_losses = []
            for _ in range(max(1, len(trans))): # < --- NEW CODE (if the value was not 1, it will help train more per episode)
            # for _ in range(max(1, len(trans) // max(1, args.train_every))): # < --- OLD CODE
                batch = buffer.sample(args.batch_size)
                loss = agent.train_on_batch(batch)
                batch_losses.append(loss)
                step += 1

                if step % args.target_sync_every == 0:
                    agent.update_target()

            ep_loss = sum(batch_losses) / len(batch_losses)
            recent_losses.append(ep_loss)

        #  FULL TensorBoard metrics
        if chief and writer is not None:
            rolling_wr = (sum(recent_outcomes) / len(recent_outcomes)
                          if recent_outcomes else 0.0)
            rolling_loss = (sum(recent_losses) / len(recent_losses)
                            if recent_losses else 0.0)

            with writer.as_default():
                tf.summary.scalar("episode/reward", ep_reward, step=ep)
                tf.summary.scalar("episode/length", ep_length, step=ep)
                tf.summary.scalar("episode/win", 1.0 if status == "bot_wins" else 0.0, step=ep)
                tf.summary.scalar("episode/win_rate_50ep", rolling_wr, step=ep)

                if ep_loss is not None:
                    tf.summary.scalar("episode/loss", ep_loss, step=ep)
                    tf.summary.scalar("episode/loss_50ep", rolling_loss, step=ep)

                tf.summary.scalar("training/epsilon", eps, step=ep)
                tf.summary.scalar("training/buffer_size", len(buffer), step=ep)
                tf.summary.scalar("training/grad_steps", step, step=ep)

        if chief and ep % args.save_every == 0:
            agent.save(args.model_dir)

            dt = time.time() - t0
            total_decided = wins + losses
            wr = wins / max(1, total_decided)

            print(f"[train_vs_mm] ep={ep} eps={eps:.3f} buf={len(buffer)} "
                f"student_wr={wr:.2%} steps={step} dt={dt:.1f}s "
                f"-> {args.model_dir}")

    if chief:
        agent.save(args.model_dir)

        _write_similar_report(games_dir, "wins", win_fp)
        _write_similar_report(games_dir, "losses", loss_fp)
        _write_similar_report(games_dir, "draws", draw_fp)

        if writer:
            writer.close()


if __name__ == "__main__":
    main()