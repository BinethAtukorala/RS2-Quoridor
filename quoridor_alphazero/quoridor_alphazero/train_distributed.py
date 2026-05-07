"""Multi-PC distributed AlphaZero trainer for 5x5 Quoridor.

Same loop as `train.py` (self-play with a persistent mp.Pool, then SGD on the
collected samples), but data-parallel across multiple machines via
`tf.distribute.MultiWorkerMirroredStrategy` — same convention as
`quoridor_ai_move/train.py`.

Each PC runs the full cycle locally:
  - its own mp.Pool of self-play workers,
  - its own ReplayBuffer fed only by its local games,
  - synchronized SGD: gradients on its local sampled batch are averaged
    (all-reduce) with the other PCs' gradients, so every PC ends each step
    with identical weights.

Only the chief (TF_CONFIG task index 0) writes TensorBoard and checkpoints.

Launch — one process per PC, one PC per cluster slot:

  # PC 0 (chief)
  TF_CONFIG='{"cluster":{"worker":["pc0:12345","pc1:12345"]},
              "task":{"type":"worker","index":0}}' \\
      python -m quoridor_alphazero.train_distributed --distributed --workers 12

  # PC 1
  TF_CONFIG='{"cluster":{"worker":["pc0:12345","pc1:12345"]},
              "task":{"type":"worker","index":1}}' \\
      python -m quoridor_alphazero.train_distributed --distributed --workers 12

Without --distributed it behaves like `train.py` on a single PC.
"""
from __future__ import annotations

import argparse
import json
import os
import time
import multiprocessing as mp
from collections import deque
from pathlib import Path

os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'

import numpy as np
import tensorflow as tf

try:
    from .mcts import MCTS
    from .network import build_az_net
    from .replay_buffer import ReplayBuffer
    from .self_play import play_game
except ImportError:
    from mcts import MCTS
    from network import build_az_net
    from replay_buffer import ReplayBuffer
    from self_play import play_game


def _atomic_save(model: tf.keras.Model, model_dir: str, keep_backups: int = 3):
    d = Path(model_dir); d.mkdir(parents=True, exist_ok=True)
    target = d / "az_net.weights.h5"
    tmp    = d / "az_net.tmp.weights.h5"
    model.save_weights(str(tmp))
    if target.exists() and keep_backups > 0:
        for i in range(keep_backups, 1, -1):
            prev = d / f"az_net.weights.bak{i-1}.h5"
            nxt  = d / f"az_net.weights.bak{i}.h5"
            if prev.exists():
                prev.replace(nxt)
        target.replace(d / "az_net.weights.bak1.h5")
    tmp.replace(target)


# Persistent worker state — populated once by the Pool initializer so
# TensorFlow + the model + MCTS only get built one time per worker process.
_WORKER = {}


def _worker_init(filters, blocks, simulations, c_puct,
                 alpha, eps, temp_moves, max_plies):
    """Pool initializer: import TF, build the model + MCTS once per worker."""
    import os
    os.environ.setdefault("TF_CPP_MIN_LOG_LEVEL", "2")
    # Workers do CPU-only self-play; hide GPUs so they don't fight the trainer
    # for VRAM and don't try to attach to a GPU that's owned by the strategy.
    os.environ.setdefault("CUDA_VISIBLE_DEVICES", "-1")

    import tensorflow as tf
    try:
        from .mcts import MCTS
        from .network import build_az_net
        from .self_play import play_game
    except ImportError:
        from mcts import MCTS
        from network import build_az_net
        from self_play import play_game

    model = build_az_net(filters=filters, n_blocks=blocks)

    @tf.function(reduce_retracing=True)
    def _forward(x):
        logits, value = model(x, training=False)
        return tf.nn.softmax(logits, axis=-1), tf.squeeze(value, axis=-1)

    def evaluator(state_batch):
        probs, value = _forward(tf.convert_to_tensor(state_batch, dtype=tf.float32))
        return probs.numpy(), value.numpy()

    mcts = MCTS(evaluator, n_simulations=simulations, c_puct=c_puct,
                dirichlet_alpha=alpha, dirichlet_eps=eps)

    _WORKER.update(
        model=model,
        mcts=mcts,
        play_game=play_game,
        temp_moves=temp_moves,
        max_plies=max_plies,
        weights_version=-1,
    )


def _worker_set_weights(payload):
    """Update the persistent worker model in-place. Returns the worker pid."""
    import os
    version, weights = payload
    if version != _WORKER["weights_version"]:
        _WORKER["model"].set_weights(weights)
        _WORKER["weights_version"] = version
    return os.getpid()


def _worker_play(_):
    """Play one self-play game with the cached model + MCTS."""
    return _WORKER["play_game"](
        _WORKER["mcts"],
        max_plies=_WORKER["max_plies"],
        temp_moves=_WORKER["temp_moves"],
    )


def _broadcast_weights(pool, n_workers, version, weights):
    """Push `weights` into every worker in the Pool."""
    payload = (version, weights)
    seen = set()
    attempts = 0
    while len(seen) < n_workers and attempts < 16:
        attempts += 1
        pending = [pool.apply_async(_worker_set_weights, (payload,))
                   for _ in range(n_workers - len(seen))]
        for r in pending:
            seen.add(r.get())


# ---------- distribution helpers (mirrors quoridor_ai_move/train.py) ----------


def make_strategy(distributed: bool) -> tf.distribute.Strategy:
    """Pick a tf.distribute strategy.

    --distributed -> MultiWorkerMirroredStrategy (requires TF_CONFIG)
    multi-GPU     -> MirroredStrategy
    otherwise     -> default (single device)
    """
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


def _is_chief() -> bool:
    tfc = os.environ.get("TF_CONFIG")
    if not tfc:
        return True
    try:
        d = json.loads(tfc)
        return d.get("task", {}).get("index", 0) == 0
    except Exception:
        return True


def _task_label() -> str:
    tfc = os.environ.get("TF_CONFIG")
    if not tfc:
        return "standalone"
    try:
        d = json.loads(tfc)
        t = d.get("task", {})
        cluster = d.get("cluster", {}).get("worker", [])
        return f"{t.get('type','worker')}:{t.get('index',0)}/{len(cluster)}"
    except Exception:
        return "unknown"


# ----------------------------- main ------------------------------------------


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("--episodes",          type=int,   default=10000)
    p.add_argument("--model-dir",         type=str,   default=str(Path.cwd() / "quoridor_az_models" / "latest"))
    p.add_argument("--resume",            action="store_true")
    p.add_argument("--simulations",       type=int,   default=64,
                   help="MCTS simulations per move during self-play.")
    p.add_argument("--c-puct",            type=float, default=1.5)
    p.add_argument("--dirichlet-alpha",   type=float, default=0.3)
    p.add_argument("--dirichlet-eps",     type=float, default=0.25,
                   help="Mix this fraction of Dirichlet noise into root priors during self-play.")
    p.add_argument("--temp-moves",        type=int,   default=12,
                   help="Number of opening plies played at temperature=1 before greedy.")
    p.add_argument("--max-plies",         type=int,   default=75)
    p.add_argument("--batch-size",        type=int,   default=256,
                   help="Per-PC SGD batch. Effective global batch = batch_size * num_workers.")
    p.add_argument("--replay-capacity",   type=int,   default=200_000)
    p.add_argument("--updates-per-game",  type=int,   default=8,
                   help="Number of SGD steps after each self-play game.")
    p.add_argument("--save-every",        type=int,   default=50, help="games")
    p.add_argument("--lr",                type=float, default=1e-3)
    p.add_argument("--lr-resume",         type=float, default=2e-4)
    p.add_argument("--filters",           type=int,   default=32)
    p.add_argument("--blocks",            type=int,   default=3)
    p.add_argument("--value-loss-weight", type=float, default=1.0)
    p.add_argument("--tb-log-dir",        type=str,   default="./quoridor_az_tensorboard/")
    p.add_argument("--workers",           type=int,   default=12,
                   help="Number of CPU cores to use for parallel self-play (per PC).")
    p.add_argument("--distributed",       action="store_true",
                   help="Multi-worker training via TF_CONFIG.")
    args = p.parse_args(argv)

    strategy = make_strategy(args.distributed)
    chief    = _is_chief()
    role     = _task_label()
    n_replicas = strategy.num_replicas_in_sync
    print(f"[az][{role}] strategy={type(strategy).__name__} replicas={n_replicas} chief={chief}")

    # Build model + optimizer inside the strategy scope so MirroredVariables
    # get created and the optimizer's slot vars are colocated with them.
    with strategy.scope():
        model = build_az_net(filters=args.filters, n_blocks=args.blocks)
        weights_path = Path(args.model_dir) / "az_net.weights.h5"
        if args.resume and weights_path.exists():
            model.load_weights(str(weights_path))
            if chief:
                print(f"[az] Resumed weights from {weights_path}")
        effective_lr = args.lr_resume if args.resume else args.lr
        optimizer = tf.keras.optimizers.Adam(learning_rate=effective_lr, clipnorm=1.0)
        # Eager build so slot variables are created inside the scope (otherwise
        # the lazy build inside the @tf.function fires outside any scope and
        # MultiWorkerMirroredStrategy raises "Need to be inside strategy.scope()").
        optimizer.build(model.trainable_variables)
    if chief:
        print(f"[az] Optimizer lr={effective_lr}")

    # Replay buffer is per-PC (samples aren't shared; only gradients are).
    buffer = ReplayBuffer(capacity=args.replay_capacity)
    buffer_path = os.path.join(args.model_dir, "replay_buffer.pkl")
    if args.resume and os.path.exists(buffer_path) and chief:
        try:
            buffer.load(buffer_path)
            print(f"[az] Resumed replay buffer (size={len(buffer)})")
        except Exception as e:
            print(f"[az] Buffer load failed: {e}")

    writer = tf.summary.create_file_writer(args.tb_log_dir) if chief else None
    if writer is not None:
        print(f"[az] TensorBoard: tensorboard --logdir={args.tb_log_dir}")

    @tf.function(reduce_retracing=True)
    def _train_step(s, pi, z):
        # Plain @tf.function under strategy.scope(): variables are mirrored,
        # so apply_gradients triggers the cross-PC all-reduce automatically.
        with tf.GradientTape() as tape:
            logits, value = model(s, training=True)
            value = tf.squeeze(value, axis=-1)
            log_p = tf.nn.log_softmax(logits, axis=-1)
            policy_loss = -tf.reduce_mean(tf.reduce_sum(pi * log_p, axis=-1))
            value_loss  = tf.reduce_mean(tf.square(z - value))
            l2_loss     = tf.add_n(model.losses) if model.losses else 0.0
            loss = policy_loss + args.value_loss_weight * value_loss + l2_loss
        grads = tape.gradient(loss, model.trainable_variables)
        optimizer.apply_gradients(zip(grads, model.trainable_variables))
        return loss, policy_loss, value_loss

    recent_lengths:  deque[int] = deque(maxlen=50)
    recent_outcomes: deque[int] = deque(maxlen=50)
    bot_wins = player_wins = draws = 0
    t0 = time.time()
    grad_step = 0

    total_batches = (args.episodes // args.workers) + 1
    total_episodes_played = 0

    print(f"[az][{role}] Starting Pool with {args.workers} workers...")

    init_args = (args.filters, args.blocks, args.simulations, args.c_puct,
                 args.dirichlet_alpha, args.dirichlet_eps,
                 args.temp_moves, args.max_plies)

    with mp.Pool(args.workers, initializer=_worker_init, initargs=init_args) as pool:
     for batch in range(1, total_batches + 1):

        # 1. Push the latest weights into every local self-play worker.
        _broadcast_weights(pool, args.workers, batch, model.get_weights())

        # 2. Run self-play in parallel on this PC.
        results = pool.map(_worker_play, [None] * args.workers)

        ep_total = ep_pol = ep_val = 0.0
        n_updates = 0

        for samples, status, ply in results:
            total_episodes_played += 1
            for s in samples:
                buffer.add(s.state, s.policy, s.z)
            recent_lengths.append(ply)
            if status == "bot_wins":
                bot_wins += 1; recent_outcomes.append(1)
            elif status == "player_wins":
                player_wins += 1; recent_outcomes.append(-1)
            else:
                draws += 1; recent_outcomes.append(0)

        # 3. Synchronized SGD: each PC samples its local batch and runs the
        #    same number of steps. apply_gradients all-reduces across PCs, so
        #    every PC ends with identical weights. NOTE: this is a barrier —
        #    if any PC's buffer is too small, ALL PCs must skip training this
        #    round, otherwise the cluster deadlocks.
        if len(buffer) >= args.batch_size:
            total_updates = args.updates_per_game * args.workers
            for _ in range(total_updates):
                s_b, pi_b, z_b = buffer.sample(args.batch_size)
                tot, pol, val = _train_step(
                    tf.convert_to_tensor(s_b),
                    tf.convert_to_tensor(pi_b),
                    tf.convert_to_tensor(z_b),
                )
                ep_total += float(tot); ep_pol += float(pol); ep_val += float(val)
                n_updates += 1; grad_step += 1
            ep_total /= n_updates; ep_pol /= n_updates; ep_val /= n_updates

        # 4. Logging + checkpointing — chief only.
        if chief and writer is not None:
            with writer.as_default():
                tf.summary.scalar("game/length",  ply, step=total_episodes_played)
                tf.summary.scalar("game/avg_length_50", float(np.mean(recent_lengths)), step=total_episodes_played)
                tf.summary.scalar("game/bot_wins",    bot_wins,    step=total_episodes_played)
                tf.summary.scalar("game/player_wins", player_wins, step=total_episodes_played)
                tf.summary.scalar("game/draws",       draws,       step=total_episodes_played)
                decisive = sum(1 for o in recent_outcomes if o != 0)
                bot_w50  = sum(1 for o in recent_outcomes if o == 1)
                tf.summary.scalar("game/bot_win_rate_50",
                                  bot_w50 / decisive if decisive > 0 else 0.0, step=total_episodes_played)
                tf.summary.scalar("game/decisive_rate_50",
                                  decisive / len(recent_outcomes), step=total_episodes_played)
                tf.summary.scalar("buffer/size", len(buffer), step=total_episodes_played)
                if n_updates > 0:
                    tf.summary.scalar("loss/total",  ep_total, step=total_episodes_played)
                    tf.summary.scalar("loss/policy", ep_pol,   step=total_episodes_played)
                    tf.summary.scalar("loss/value",  ep_val,   step=total_episodes_played)
                    tf.summary.scalar("training/grad_steps", grad_step, step=total_episodes_played)

        if chief and (total_episodes_played % args.save_every < args.workers
                      or total_episodes_played >= args.save_every):
            _atomic_save(model, args.model_dir)
            try: buffer.save(buffer_path)
            except Exception as e: print(f"[az] Buffer save failed: {e}")
            dt = time.time() - t0
            print(f"[az][{role}] eps={total_episodes_played} ply={np.mean(recent_lengths):.1f} buf={len(buffer)} "
                  f"bot={bot_wins} player={player_wins} draws={draws} "
                  f"loss={ep_total:.3f} (p={ep_pol:.3f} v={ep_val:.3f}) "
                  f"dt={dt:.1f}s")

    if chief:
        _atomic_save(model, args.model_dir)
        try: buffer.save(buffer_path)
        except Exception: pass
        if writer is not None:
            writer.close()
        print(f"[az] Final save -> {args.model_dir}")


if __name__ == "__main__":
    mp.set_start_method('spawn', force=True)
    main()
