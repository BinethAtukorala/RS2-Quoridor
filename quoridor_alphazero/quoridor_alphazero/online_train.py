"""Targeted background trainer fired after a human game loss.

Loads the latest weights, plays N short self-play games starting from a
critical position from the lost game, runs a small number of gradient
steps, and atomic-saves new weights so ai_move_node hot-reloads them.

Single process, CPU-only — designed to run alongside the live AI without
fighting it for the GPU.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

# Online trainer uses the GPU: batch=256 gradient updates benefit from it,
# and the live node (which uses CPU) won't fight us for it.
os.environ.setdefault("TF_CPP_MIN_LOG_LEVEL", "2")

import numpy as np
import tensorflow as tf

# Don't grab all VRAM — the node may be running too.
for g in tf.config.list_physical_devices("GPU"):
    try: tf.config.experimental.set_memory_growth(g, True)
    except Exception: pass

from quoridor_game.quoridor_utils import QuoridorBoard

try:
    from .encoder import action_to_move, encode_state
    from .mcts import MCTS, visit_counts_to_policy
    from .network import build_az_net
    from .replay_buffer import ReplayBuffer
    from .self_play import Sample
except ImportError:
    from encoder import action_to_move, encode_state
    from mcts import MCTS, visit_counts_to_policy
    from network import build_az_net
    from replay_buffer import ReplayBuffer
    from self_play import Sample


def _outcome_value(status: str, side: str) -> float:
    if status == "bot_wins":
        return 1.0 if side == "bot" else -1.0
    if status == "player_wins":
        return 1.0 if side == "player" else -1.0
    return 0.0


def play_game_from(mcts: MCTS, start_board: QuoridorBoard,
                   max_plies: int = 75,
                   temp_moves: int = 6,
                   temp_high: float = 1.0):
    """Self-play starting from `start_board` instead of the initial position."""
    board = start_board.copy()
    samples: list[Sample] = []
    ply = 0
    while board.game_status == "in_progress" and ply < max_plies:
        side = board.current_turn
        counts, _root = mcts.run(board)
        if counts.sum() == 0:
            break

        temp = temp_high if ply < temp_moves else 0.0
        policy_target = visit_counts_to_policy(counts, temperature=temp_high)
        play_dist = visit_counts_to_policy(counts, temperature=temp)

        samples.append(Sample(
            state=encode_state(board, side),
            policy=policy_target,
            side=side,
        ))

        if temp <= 1e-6:
            action = int(np.argmax(play_dist))
        else:
            action = int(np.random.choice(len(play_dist), p=play_dist))
        move = action_to_move(board, action, side)
        if move is None or not board.apply_move(move):
            break
        ply += 1

    status = board.game_status
    for s in samples:
        s.z = _outcome_value(status, s.side)
    return samples, status, ply


def _atomic_save(model, model_dir: str):
    d = Path(model_dir)
    d.mkdir(parents=True, exist_ok=True)
    target = d / "az_net.weights.h5"
    tmp = d / "az_net.tmp.weights.h5"
    model.save_weights(str(tmp))
    tmp.replace(target)


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("--model-dir", required=True)
    p.add_argument("--start-board", required=True,
                   help="Path to a JSON file with the QuoridorBoard state to start self-play from.")
    p.add_argument("--filters", type=int, default=32)
    p.add_argument("--blocks", type=int, default=3)
    p.add_argument("--episodes", type=int, default=40)
    p.add_argument("--simulations", type=int, default=64)
    p.add_argument("--c-puct", type=float, default=1.5)
    p.add_argument("--dirichlet-alpha", type=float, default=0.3)
    p.add_argument("--dirichlet-eps", type=float, default=0.25)
    p.add_argument("--temp-moves", type=int, default=6)
    p.add_argument("--max-plies", type=int, default=75)
    p.add_argument("--batch-size", type=int, default=256)
    p.add_argument("--updates", type=int, default=400)
    p.add_argument("--lr", type=float, default=1e-4)
    args = p.parse_args(argv)

    with open(args.start_board) as f:
        start_board = QuoridorBoard.from_json(f.read())

    model = build_az_net(filters=args.filters, n_blocks=args.blocks)
    weights_path = Path(args.model_dir) / "az_net.weights.h5"
    if weights_path.exists():
        model.load_weights(str(weights_path))
        print(f"[online] Loaded weights from {weights_path}", flush=True)
    else:
        print(f"[online] No weights at {weights_path}; aborting", flush=True)
        return

    optimizer = tf.keras.optimizers.Adam(learning_rate=args.lr, clipnorm=1.0)

    @tf.function(reduce_retracing=True)
    def _forward(x):
        logits, value = model(x, training=False)
        return tf.nn.softmax(logits, axis=-1), tf.squeeze(value, axis=-1)

    def evaluator(state_batch):
        probs, value = _forward(tf.convert_to_tensor(state_batch, dtype=tf.float32))
        return probs.numpy(), value.numpy()

    mcts = MCTS(evaluator,
                n_simulations=args.simulations,
                c_puct=args.c_puct,
                dirichlet_alpha=args.dirichlet_alpha,
                dirichlet_eps=args.dirichlet_eps)

    # Load main buffer if available so gradient steps see a healthy mix.
    buffer = ReplayBuffer(capacity=200_000)
    main_buf_path = os.path.join(args.model_dir, "replay_buffer.pkl")
    if os.path.exists(main_buf_path):
        try:
            buffer.load(main_buf_path)
            print(f"[online] Loaded main buffer ({len(buffer)} samples)", flush=True)
        except Exception as e:
            print(f"[online] Buffer load failed: {e}", flush=True)

    targeted: list[Sample] = []
    for ep in range(args.episodes):
        samples, status, ply = play_game_from(
            mcts, start_board,
            max_plies=args.max_plies,
            temp_moves=args.temp_moves)
        targeted.extend(samples)
        for s in samples:
            buffer.add(s.state, s.policy, s.z)
        print(f"[online] ep={ep+1}/{args.episodes} status={status} ply={ply} "
              f"new_samples={len(samples)}", flush=True)

    if len(buffer) < args.batch_size:
        print("[online] buffer too small; skipping gradient updates", flush=True)
        return

    @tf.function(reduce_retracing=True)
    def _train_step(s, pi, z):
        with tf.GradientTape() as tape:
            logits, value = model(s, training=True)
            value = tf.squeeze(value, axis=-1)
            log_p = tf.nn.log_softmax(logits, axis=-1)
            policy_loss = -tf.reduce_mean(tf.reduce_sum(pi * log_p, axis=-1))
            value_loss = tf.reduce_mean(tf.square(z - value))
            l2_loss = tf.add_n(model.losses) if model.losses else 0.0
            loss = policy_loss + value_loss + l2_loss
        grads = tape.gradient(loss, model.trainable_variables)
        optimizer.apply_gradients(zip(grads, model.trainable_variables))
        return loss, policy_loss, value_loss

    # Build an upweighted "targeted" array so each batch includes ~25% of
    # the new samples even though they're a tiny fraction of the buffer.
    if targeted:
        t_states = np.stack([s.state for s in targeted])
        t_pol = np.stack([s.policy for s in targeted])
        t_val = np.array([s.z for s in targeted], dtype=np.float32)
    else:
        t_states = t_pol = t_val = None

    n_targeted_per_batch = max(1, args.batch_size // 4) if t_states is not None else 0
    n_main_per_batch = args.batch_size - n_targeted_per_batch

    for step in range(args.updates):
        s_main, pi_main, z_main = buffer.sample(n_main_per_batch)
        if t_states is not None:
            idx = np.random.randint(0, len(t_states), size=n_targeted_per_batch)
            s_b = np.concatenate([s_main, t_states[idx]])
            pi_b = np.concatenate([pi_main, t_pol[idx]])
            z_b = np.concatenate([z_main, t_val[idx]])
        else:
            s_b, pi_b, z_b = s_main, pi_main, z_main
        tot, pol, val = _train_step(
            tf.convert_to_tensor(s_b),
            tf.convert_to_tensor(pi_b),
            tf.convert_to_tensor(z_b))
        if (step + 1) % 50 == 0:
            print(f"[online] step={step+1}/{args.updates} "
                  f"loss={float(tot):.3f} (p={float(pol):.3f} v={float(val):.3f})",
                  flush=True)

    _atomic_save(model, args.model_dir)
    print(f"[online] Saved updated weights -> {weights_path}", flush=True)


if __name__ == "__main__":
    main()
