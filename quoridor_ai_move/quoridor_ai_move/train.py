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

import numpy as np
import tensorflow as tf

# from .agent import DQNAgent
# from .replay_buffer import ReplayBuffer
# from .self_play import play_game

try:
    from .agent import DQNAgent
    from .replay_buffer import ReplayBuffer
    from .self_play import play_game
except ImportError:
    from agent import DQNAgent
    from replay_buffer import ReplayBuffer
    from self_play import play_game


def make_strategy(distributed: bool) -> tf.distribute.Strategy:
    # Choose the appropriate tf.distribute strategy for the available hardware.
    # Multi-worker requires TF_CONFIG to be set in the environment.
    gpus = tf.config.list_physical_devices("GPU")
    for g in gpus:
        try:
            tf.config.experimental.set_memory_growth(g, True)
        except Exception:
            pass
    if distributed:
        if "TF_CONFIG" not in os.environ:
            raise RuntimeError("Set TF_CONFIG env var to use --distributed.")
        print(f"[train] Multi-worker mode, TF_CONFIG={os.environ['TF_CONFIG']}")
        return tf.distribute.MultiWorkerMirroredStrategy()
    if len(gpus) > 1:
        print(f"[train] MirroredStrategy across {len(gpus)} GPUs")
        return tf.distribute.MirroredStrategy()
    print(f"[train] Default strategy (GPUs available: {len(gpus)})")
    return tf.distribute.get_strategy()


def _epsilon_greedy_factory(agent: DQNAgent, eps_ref: list[float]):
    # eps_ref is a single-element list so the outer training loop can mutate
    # epsilon (decay it over episodes) without rebuilding the closure.
    def policy(board, side, mask):
        # from .encoder import encode_state
        
        try:
            from .encoder import encode_state
        except ImportError:
            from encoder import encode_state

        s = encode_state(board, side)
        return agent.select_action(s, mask, eps_ref[0])
    return policy


def _is_chief() -> bool:
    # In multi-worker training, only the chief (task index 0) writes
    # checkpoints / logs. Single-process runs are trivially the chief.
    tfc = os.environ.get("TF_CONFIG")
    if not tfc:
        return True
    try:
        d = json.loads(tfc)
        return d.get("task", {}).get("index", 0) == 0
    except Exception:
        return True


def main(argv=None):
    # CLI hyperparameters: episodes, learning rate, replay capacity, the
    # epsilon decay schedule, model architecture knobs, and where to save.
    p = argparse.ArgumentParser()
    p.add_argument("--episodes", type=int, default=2000)
    p.add_argument("--model-dir", type=str, default=str(Path.home() / "quoridor_models" / "latest"))
    p.add_argument("--resume", action="store_true", help="Load existing weights before training")
    p.add_argument("--batch-size", type=int, default=256)
    p.add_argument("--replay-capacity", type=int, default=200_000)
    p.add_argument("--train-every", type=int, default=1, help="Train step every N plies")
    p.add_argument("--target-sync-every", type=int, default=500)
    p.add_argument("--save-every", type=int, default=100, help="episodes")
    p.add_argument("--lr", type=float, default=1e-3)
    p.add_argument("--gamma", type=float, default=0.99)
    p.add_argument("--eps-start", type=float, default=1.0)
    p.add_argument("--eps-end", type=float, default=0.05)
    p.add_argument("--eps-decay-episodes", type=int, default=1500)
    p.add_argument("--distributed", action="store_true")
    p.add_argument("--filters", type=int, default=64)
    p.add_argument("--blocks", type=int, default=4)
    p.add_argument("--tau", type=float, default=0.005,
               help="Soft target update rate. Set to 1.0 for hard copy.")
    # ADDED: argument to set tensorboard log directory
    p.add_argument("--tb-log-dir", type=str, default="./quoridor_tensorboard/")
    args = p.parse_args(argv)

    strategy = make_strategy(args.distributed)
    agent = DQNAgent(lr=args.lr, gamma=args.gamma, strategy=strategy,
                     filters=args.filters, n_blocks=args.blocks)
    if args.resume and agent.load(args.model_dir):
        print(f"[train] Resumed weights from {args.model_dir}")

    buffer = ReplayBuffer(capacity=args.replay_capacity)
    eps_ref = [args.eps_start]
    # Self-play: agent plays both sides (records both so we double data per game).
    policy = _epsilon_greedy_factory(agent, eps_ref)

    chief = _is_chief()
    step = 0
    wins = losses = draws = 0
    t0 = time.time()

    # ADDED: create the TensorBoard writer (only on the chief node)
    writer = tf.summary.create_file_writer(args.tb_log_dir) if chief else None
    if writer:
        print(f"[train] TensorBoard logs -> {args.tb_log_dir}")
        print(f"[train] Run: tensorboard --logdir={args.tb_log_dir}")

    # ADDED: rolling window to track recent losses for smoothing
    recent_losses = []
    WINDOW = 50  # average loss over last 50 training steps

    for ep in range(1, args.episodes + 1):
        # Linear epsilon decay from eps_start to eps_end across the schedule;
        # clamped at eps_end thereafter for residual exploration.
        eps = max(args.eps_end,
                  args.eps_start - (args.eps_start - args.eps_end) *
                  (ep / max(1, args.eps_decay_episodes)))
        eps_ref[0] = eps

        # Generate one self-play game and dump every transition into the buffer.
        trans, status = play_game(policy, policy, record_sides=("bot", "player"))
        for t in trans:
            buffer.add(t.state, t.action, t.reward, t.next_state, t.done, t.next_mask)

        if status == "bot_wins":
            wins += 1
        elif status == "player_wins":
            losses += 1
        else:
            draws += 1

        # # ADDED: track episode-level metrics
        # ep_reward = sum(t.reward for t in trans)

        # # Instead of summing all transitions (bot + player cancel out):
        # ep_reward = sum(t.reward for t in trans if t.reward != 0.0)
        # Or better, track win/loss as +1/-1 explicitly:
        ep_reward = 1.0 if status == "bot_wins" else (-1.0 if status == "player_wins" else 0.0)

        ep_length = len(trans)
        ep_loss = None 

        # Run several gradient updates per game once the buffer is warm.
        # Periodically copy the online network into the target network.
        if len(buffer) >= args.batch_size:
            batch_losses = []
            for _ in range(max(1, len(trans) // max(1, args.train_every))):
                batch = buffer.sample(args.batch_size)
                loss = agent.train_on_batch(batch) # Returns float loss
                batch_losses.append(loss)
                step += 1

                if step % args.target_sync_every == 0:
                    agent.update_target()

                # agent.update_target()

            ep_loss = sum(batch_losses) / len(batch_losses)
            recent_losses.append(ep_loss)
            if len(recent_losses) > WINDOW:
                recent_losses.pop(0)

        # ADDED: write all metrics to TensorBoard every episode (chief only)
        if chief and writer is not None:
            with writer.as_default():
                tf.summary.scalar("episode/reward", ep_reward, step=ep)
                tf.summary.scalar("episode/length_plies", ep_length, step=ep)

                total_decided = wins + losses
                if total_decided > 0:
                    win_rate = wins / total_decided
                    tf.summary.scalar("episode/win_rate", win_rate, step=ep)

                tf.summary.scalar("train/epsilon", eps, step=ep)
                tf.summary.scalar("train/buffer_size", len(buffer), step=ep)

                if ep_loss is not None:
                    tf.summary.scalar("train/loss", ep_loss, step=ep)
                    if len(recent_losses) >= 10:
                        smoothed = sum(recent_losses) / len(recent_losses)
                        tf.summary.scalar("train/loss_smoothed", smoothed, step=ep)
            writer.flush()

        # Periodic checkpointing + progress log (chief only in multi-worker).
        if chief and ep % args.save_every == 0:
            agent.save(args.model_dir)
            dt = time.time() - t0
            print(f"[train] ep={ep} eps={eps:.3f} buf={len(buffer)} "
                  f"wins={wins} losses={losses} draws={draws} "
                  f"steps={step} dt={dt:.1f}s -> saved to {args.model_dir}")

    if chief:
        agent.save(args.model_dir)
        print(f"[train] Final save to {args.model_dir}")
        if writer:
            writer.close()


if __name__ == "__main__":
    main()