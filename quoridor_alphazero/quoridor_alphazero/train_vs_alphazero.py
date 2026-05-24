"""Adversarial training: train a Challenger model to beat a frozen AlphaZero.

Strategy
--------
Instead of pure self-play, the Challenger plays exclusively against the frozen
AlphaZero opponent. This means every training signal comes from positions where
the Challenger either beat or lost to a strong, fixed adversary — no echo-chamber
self-play collapse.

The training loop:
  1. Run N parallel games: Challenger (MCTS) vs AlphaZero (MCTS, frozen).
  2. Collect samples ONLY from the Challenger's moves (we're training it, not AZ).
  3. Push samples into the Challenger's replay buffer.
  4. Run SGD steps on the Challenger network.
  5. Every `--eval-every` episodes, run a tournament (no MCTS noise) and log
     the Challenger's win rate vs AlphaZero.
  6. Save the Challenger whenever it sets a new win-rate record.

Key differences from vanilla AlphaZero self-play
-------------------------------------------------
- AlphaZero weights are loaded once and NEVER updated.
- Challenger uses more Dirichlet noise (eps=0.35 default) since it must
  discover moves the AZ never uses — pure exploitation fails here.
- Value targets are real game outcomes, not bootstrapped (same as AZ).
- Optional curriculum: start AZ at fewer simulations and ramp up as the
  Challenger improves (--az-sim-start / --az-sim-max / --az-sim-ramp).
"""
from __future__ import annotations

import argparse
import json
import multiprocessing as mp
import os
import time
from collections import deque
from pathlib import Path

os.environ["TF_CPP_MIN_LOG_LEVEL"] = "2"

import numpy as np
import tensorflow as tf

# ---------------------------------------------------------------------------
# Imports — compatible with both package-installed and standalone runs.
# ---------------------------------------------------------------------------
try:
    from quoridor_alphazero.encoder import (
        NUM_ACTIONS, action_to_move, encode_state, legal_action_mask,
    )
    from quoridor_alphazero.mcts import MCTS, visit_counts_to_policy
    from quoridor_alphazero.network import build_az_net
    from quoridor_alphazero.replay_buffer import ReplayBuffer
except ImportError:
    from encoder import (
        NUM_ACTIONS, action_to_move, encode_state, legal_action_mask,
    )
    from mcts import MCTS, visit_counts_to_policy
    from network import build_az_net
    from replay_buffer import ReplayBuffer

try:
    from quoridor_game.quoridor_utils import QuoridorBoard
except ImportError:
    from quoridor_utils import QuoridorBoard


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _atomic_save(model: tf.keras.Model, model_dir: str, keep_backups: int = 3):
    d = Path(model_dir)
    d.mkdir(parents=True, exist_ok=True)
    target = d / "challenger.weights.h5"
    tmp    = d / "challenger.tmp.weights.h5"
    model.save_weights(str(tmp))
    if target.exists() and keep_backups > 0:
        for i in range(keep_backups, 1, -1):
            prev = d / f"challenger.weights.bak{i-1}.h5"
            nxt  = d / f"challenger.weights.bak{i}.h5"
            if prev.exists():
                prev.replace(nxt)
        target.replace(d / "challenger.weights.bak1.h5")
    tmp.replace(target)


def _build_evaluator(model: tf.keras.Model):
    @tf.function(reduce_retracing=True)
    def _forward(x):
        logits, value = model(x, training=False)
        return tf.nn.softmax(logits, axis=-1), tf.squeeze(value, axis=-1)

    def evaluator(state_batch: np.ndarray):
        probs, value = _forward(tf.convert_to_tensor(state_batch, dtype=tf.float32))
        return probs.numpy(), value.numpy()

    return evaluator


def _outcome(status: str, side: str) -> float:
    if status == "bot_wins":
        return 1.0 if side == "bot" else -1.0
    if status == "player_wins":
        return 1.0 if side == "player" else -1.0
    return 0.0


# ---------------------------------------------------------------------------
# Worker: one game, Challenger vs AlphaZero
# ---------------------------------------------------------------------------

def _game_worker(args: dict) -> dict:
    """
    Spawned in a child process. Plays one full game.

    Returns a dict with:
      - samples: list of (state, policy, z) for the Challenger's moves only
      - status: final game_status string
      - ply: number of half-moves played
      - challenger_side: "bot" or "player"
    """
    # ---- unpack ----
    challenger_weights = args["challenger_weights"]
    az_weights         = args["az_weights"]
    challenger_side    = args["challenger_side"]   # "bot" or "player"
    filters            = args["filters"]
    blocks             = args["blocks"]
    ch_sims            = args["ch_sims"]
    az_sims            = args["az_sims"]
    c_puct             = args["c_puct"]
    dirichlet_alpha    = args["dirichlet_alpha"]
    dirichlet_eps      = args["dirichlet_eps"]
    temp_moves         = args["temp_moves"]
    max_plies          = args["max_plies"]
    eval_mode          = args["eval_mode"]         # True → no noise, greedy

    import os
    os.environ["TF_CPP_MIN_LOG_LEVEL"] = "2"
    import tensorflow as tf
    import numpy as np

    try:
        from quoridor_alphazero.encoder import action_to_move, encode_state
        from quoridor_alphazero.mcts import MCTS, visit_counts_to_policy
        from quoridor_alphazero.network import build_az_net
    except ImportError:
        from encoder import action_to_move, encode_state
        from mcts import MCTS, visit_counts_to_policy
        from network import build_az_net

    try:
        from quoridor_game.quoridor_utils import QuoridorBoard
    except ImportError:
        from quoridor_utils import QuoridorBoard

    # Limit GPU memory in workers
    for g in tf.config.list_physical_devices("GPU"):
        try:
            tf.config.experimental.set_memory_growth(g, True)
        except Exception:
            pass

    # Build both models
    challenger_model = build_az_net(filters=filters, n_blocks=blocks)
    challenger_model.set_weights(challenger_weights)

    az_model = build_az_net(filters=filters, n_blocks=blocks)
    az_model.set_weights(az_weights)

    def _make_eval(model):
        @tf.function(reduce_retracing=True)
        def _fwd(x):
            logits, v = model(x, training=False)
            return tf.nn.softmax(logits, axis=-1), tf.squeeze(v, axis=-1)
        def ev(sb):
            p, v = _fwd(tf.convert_to_tensor(sb, dtype=tf.float32))
            return p.numpy(), v.numpy()
        return ev

    ch_eps  = 0.0 if eval_mode else dirichlet_eps
    ch_mcts = MCTS(_make_eval(challenger_model),
                   n_simulations=ch_sims,
                   c_puct=c_puct,
                   dirichlet_alpha=dirichlet_alpha,
                   dirichlet_eps=ch_eps)

    az_mcts = MCTS(_make_eval(az_model),
                   n_simulations=az_sims,
                   c_puct=c_puct,
                   dirichlet_alpha=dirichlet_alpha,
                   dirichlet_eps=0.0)   # AZ is always deterministic

    board = QuoridorBoard(n=5)
    samples_raw = []   # (state, policy_target, side)
    ply = 0

    while board.game_status == "in_progress" and ply < max_plies:
        side = board.current_turn
        is_challenger = (side == challenger_side)
        mcts_engine   = ch_mcts if is_challenger else az_mcts

        counts, _ = mcts_engine.run(board)
        if counts.sum() == 0:
            break

        temp = (1.0 if ply < temp_moves else 0.0) if not eval_mode else 0.0
        policy_target = visit_counts_to_policy(counts, temperature=1.0)
        play_dist     = visit_counts_to_policy(counts, temperature=temp)

        # Only record samples for the Challenger
        if is_challenger:
            samples_raw.append((encode_state(board, side), policy_target, side))

        if temp <= 1e-6:
            action = int(np.argmax(play_dist))
        else:
            action = int(np.random.choice(len(play_dist), p=play_dist))

        move = action_to_move(board, action, side)
        if move is None or not board.apply_move(move):
            break
        ply += 1

    status = board.game_status
    z_map = {s: (1.0 if (
        (status == "bot_wins"    and s == "bot") or
        (status == "player_wins" and s == "player")
    ) else (-1.0 if status != "in_progress" else 0.0))
        for s in ("bot", "player")}

    samples = [
        (state, policy, float(z_map[s]))
        for (state, policy, s) in samples_raw
    ]

    return {
        "samples":        samples,
        "status":         status,
        "ply":            ply,
        "challenger_side": challenger_side,
    }


# ---------------------------------------------------------------------------
# Main training loop
# ---------------------------------------------------------------------------

def main(argv=None):
    p = argparse.ArgumentParser(
        description="Train a Challenger model to beat a frozen AlphaZero."
    )
    # Paths
    p.add_argument("--az-model-dir",      type=str,
                   default=str(Path.cwd() / "quoridor_az_models" / "latest"),
                   help="Directory containing az_net.weights.h5 (frozen opponent).")
    p.add_argument("--challenger-dir",    type=str,
                   default=str(Path.cwd() / "quoridor_challenger_models" / "latest"),
                   help="Where to save challenger weights.")
    p.add_argument("--resume",            action="store_true",
                   help="Resume challenger from --challenger-dir if weights exist.")
    p.add_argument("--flush-buffer",      action="store_true",
                   help="Discard the loaded replay buffer contents on resume. Use this "
                        "when resuming after a bug fix so stale data does not persist.")

    # Training scale
    p.add_argument("--episodes",          type=int, default=20_000)
    p.add_argument("--workers",           type=int,
                   default=max(1, (os.cpu_count() or 4) - 1))
    p.add_argument("--batch-size",        type=int, default=256)
    p.add_argument("--replay-capacity",   type=int, default=30_000,
                   help="Keep this small early on (~30k) so the buffer stays fresh. "
                        "Stale samples from random early play poison gradients.")
    p.add_argument("--updates-per-game",  type=int, default=20,
                   help="Gradient steps per worker game. Higher = more training signal "
                        "per batch relative to self-play games collected.")
    p.add_argument("--save-every",        type=int, default=50,
                   help="Save challenger every N episodes.")

    # Network
    p.add_argument("--filters",           type=int, default=32)
    p.add_argument("--blocks",            type=int, default=3)

    # Challenger MCTS
    p.add_argument("--ch-sims",           type=int, default=64,
                   help="Challenger MCTS simulations per move.")
    p.add_argument("--c-puct",            type=float, default=1.5)
    p.add_argument("--dirichlet-alpha",   type=float, default=0.3)
    p.add_argument("--dirichlet-eps",     type=float, default=0.35,
                   help="Higher than AZ default — challenger needs more exploration.")

    # AlphaZero MCTS (curriculum)
    p.add_argument("--az-sims",           type=int, default=64,
                   help="AZ simulations per move. Set equal to --ch-sims for fair play, "
                        "or lower initially for curriculum.")
    p.add_argument("--az-sim-start",      type=int, default=None,
                   help="Curriculum: start AZ at this many sims (overrides --az-sims).")
    p.add_argument("--az-sim-max",        type=int, default=None,
                   help="Curriculum: ramp AZ up to this many sims by end of training.")
    p.add_argument("--az-sim-ramp",       type=int, default=5_000,
                   help="Curriculum: ramp AZ sims linearly over this many episodes.")

    # Self-play parameters
    p.add_argument("--temp-moves",        type=int, default=12)
    p.add_argument("--max-plies",         type=int, default=150)

    # Optimiser
    p.add_argument("--lr",                type=float, default=1e-3)
    p.add_argument("--lr-resume",         type=float, default=2e-4)
    p.add_argument("--value-loss-weight", type=float, default=0.3,
                   help="Weight on value MSE loss. Keep low (0.3) early — the value head "
                        "trivially learns 'always -1' and can swamp the policy gradient.")

    # Evaluation
    p.add_argument("--eval-every",        type=int, default=200,
                   help="Run a tournament every N episodes.")
    p.add_argument("--eval-games",        type=int, default=40,
                   help="Number of games per evaluation tournament.")
    p.add_argument("--eval-sims",         type=int, default=200,
                   help="MCTS sims per move during evaluation (heavier = more accurate).")

    # Misc
    p.add_argument("--tb-log-dir",        type=str,
                   default="./quoridor_challenger_tensorboard/")

    args = p.parse_args(argv)

    # ---- GPU setup ----
    for g in tf.config.list_physical_devices("GPU"):
        try:
            tf.config.experimental.set_memory_growth(g, True)
        except Exception:
            pass

    # ---- Load frozen AlphaZero ----
    az_weights_path = Path(args.az_model_dir) / "az_net.weights.h5"
    if not az_weights_path.exists():
        raise FileNotFoundError(
            f"AlphaZero weights not found at {az_weights_path}.\n"
            f"Train the base model first with train.py, or set --az-model-dir."
        )
    az_model = build_az_net(filters=args.filters, n_blocks=args.blocks)
    az_model.load_weights(str(az_weights_path))
    az_weights_snapshot = az_model.get_weights()   # frozen forever
    print(f"[challenger] Loaded frozen AlphaZero from {az_weights_path}")

    # ---- Build Challenger ----
    challenger = build_az_net(filters=args.filters, n_blocks=args.blocks)
    ch_weights_path = Path(args.challenger_dir) / "challenger.weights.h5"
    if args.resume and ch_weights_path.exists():
        challenger.load_weights(str(ch_weights_path))
        print(f"[challenger] Resumed from {ch_weights_path}")
    effective_lr = args.lr_resume if args.resume else args.lr
    optimizer = tf.keras.optimizers.Adam(learning_rate=effective_lr, clipnorm=1.0)
    print(f"[challenger] Optimizer lr={effective_lr}")

    # ---- Replay buffer ----
    buffer = ReplayBuffer(capacity=args.replay_capacity)
    buffer_path = str(Path(args.challenger_dir) / "replay_buffer.pkl")
    if args.resume and os.path.exists(buffer_path) and not args.flush_buffer:
        try:
            buffer.load(buffer_path)
            print(f"[challenger] Resumed replay buffer (size={len(buffer)})")
        except Exception as e:
            print(f"[challenger] Buffer load failed: {e}")
    elif args.flush_buffer:
        print(f"[challenger] --flush-buffer set: starting with empty replay buffer.")

    # ---- TensorBoard ----
    writer = tf.summary.create_file_writer(args.tb_log_dir)
    print(f"[challenger] TensorBoard: tensorboard --logdir={args.tb_log_dir}")

    # ---- Training step (compiled) ----
    @tf.function(reduce_retracing=True)
    def _train_step(s, pi, z):
        with tf.GradientTape() as tape:
            logits, value = challenger(s, training=True)
            value = tf.squeeze(value, axis=-1)
            log_p = tf.nn.log_softmax(logits, axis=-1)
            policy_loss = -tf.reduce_mean(tf.reduce_sum(pi * log_p, axis=-1))
            value_loss  = tf.reduce_mean(tf.square(z - value))
            l2_loss     = tf.add_n(challenger.losses) if challenger.losses else 0.0
            loss        = policy_loss + args.value_loss_weight * value_loss + l2_loss
        grads = tape.gradient(loss, challenger.trainable_variables)
        optimizer.apply_gradients(zip(grads, challenger.trainable_variables))
        return loss, policy_loss, value_loss

    # ---- Curriculum: compute AZ sims for a given episode count ----
    def _az_sims_for(ep: int) -> int:
        if args.az_sim_start is None:
            return args.az_sims
        start = args.az_sim_start
        end   = args.az_sim_max or args.az_sims
        ramp  = args.az_sim_ramp
        frac  = min(1.0, ep / max(1, ramp))
        return int(start + frac * (end - start))

    # ---- State tracking ----
    recent_lengths:   deque[int]   = deque(maxlen=100)
    recent_outcomes:  deque[float] = deque(maxlen=100)  # +1 = ch win, -1 = ch loss
    best_win_rate     = 0.0
    total_episodes    = 0
    grad_step         = 0
    ch_wins = ch_losses = draws = 0
    t0 = time.time()

    total_batches = (args.episodes // args.workers) + 1

    print(f"[challenger] Starting training: {args.episodes} episodes, "
          f"{args.workers} workers, Challenger sims={args.ch_sims}, "
          f"AZ sims={args.az_sims}")
    print(f"[challenger] Challenger plays alternating sides each batch.")

    pool = mp.Pool(args.workers)
    try:
        for batch in range(1, total_batches + 1):
            if total_episodes >= args.episodes:
                break

            az_sims_now = _az_sims_for(total_episodes)

            # Alternate which side the Challenger plays each batch so it
            # learns both opening positions equally.
            ch_side = "bot" if batch % 2 == 1 else "player"

            ch_weights_now = challenger.get_weights()
            worker_args_list = [
                {
                    "challenger_weights": ch_weights_now,
                    "az_weights":         az_weights_snapshot,
                    "challenger_side":    ch_side,
                    "filters":            args.filters,
                    "blocks":             args.blocks,
                    "ch_sims":            args.ch_sims,
                    "az_sims":            az_sims_now,
                    "c_puct":             args.c_puct,
                    "dirichlet_alpha":    args.dirichlet_alpha,
                    "dirichlet_eps":      args.dirichlet_eps,
                    "temp_moves":         args.temp_moves,
                    "max_plies":          args.max_plies,
                    "eval_mode":          False,
                }
                for _ in range(args.workers)
            ]

            results = pool.map(_game_worker, worker_args_list)

            ep_total = ep_pol = ep_val = 0.0
            n_updates = 0

            for res in results:
                total_episodes += 1
                samples  = res["samples"]
                status   = res["status"]
                ply      = res["ply"]
                ch_side_ = res["challenger_side"]

                for state, policy, z in samples:
                    buffer.add(state, policy, z)

                recent_lengths.append(ply)

                # Determine outcome from Challenger's perspective
                if status == "bot_wins":
                    outcome = 1.0 if ch_side_ == "bot" else -1.0
                elif status == "player_wins":
                    outcome = 1.0 if ch_side_ == "player" else -1.0
                else:
                    outcome = 0.0

                recent_outcomes.append(outcome)
                if outcome > 0:
                    ch_wins += 1
                elif outcome < 0:
                    ch_losses += 1
                else:
                    draws += 1

            # Train
            if len(buffer) >= args.batch_size:
                # Staleness check: warn if buffer is mostly old data.
                # If capacity >> samples collected, gradients are dominated by early random games.
                staleness = len(buffer) / max(1, args.replay_capacity)
                if staleness < 0.3 and total_episodes % (args.save_every * 4) < args.workers:
                    print(f"[challenger] Buffer only {staleness:.0%} full "
                          f"({len(buffer)}/{args.replay_capacity}). "
                          f"Gradients may be noisy — consider --replay-capacity {len(buffer) * 2}.")
                total_updates = args.updates_per_game * args.workers
                for _ in range(total_updates):
                    s_b, pi_b, z_b = buffer.sample(args.batch_size)
                    tot, pol, val = _train_step(
                        tf.convert_to_tensor(s_b),
                        tf.convert_to_tensor(pi_b),
                        tf.convert_to_tensor(z_b),
                    )
                    ep_total += float(tot)
                    ep_pol   += float(pol)
                    ep_val   += float(val)
                    n_updates += 1
                    grad_step += 1
                ep_total /= n_updates
                ep_pol   /= n_updates
                ep_val   /= n_updates

            # ---- TensorBoard logging ----
            decisive  = sum(1 for o in recent_outcomes if o != 0)
            ch_wins50 = sum(1 for o in recent_outcomes if o > 0)
            win_rate  = ch_wins50 / decisive if decisive > 0 else 0.0

            with writer.as_default():
                tf.summary.scalar("game/length",        ply,  step=total_episodes)
                tf.summary.scalar("game/avg_length_100",
                                  float(np.mean(recent_lengths)), step=total_episodes)
                tf.summary.scalar("challenger/wins",    ch_wins,   step=total_episodes)
                tf.summary.scalar("challenger/losses",  ch_losses, step=total_episodes)
                tf.summary.scalar("challenger/draws",   draws,     step=total_episodes)
                tf.summary.scalar("challenger/win_rate_100", win_rate, step=total_episodes)
                tf.summary.scalar("challenger/az_sims_now",  az_sims_now, step=total_episodes)
                tf.summary.scalar("buffer/size",        len(buffer), step=total_episodes)
                if n_updates > 0:
                    tf.summary.scalar("loss/total",   ep_total, step=total_episodes)
                    tf.summary.scalar("loss/policy",  ep_pol,   step=total_episodes)
                    tf.summary.scalar("loss/value",   ep_val,   step=total_episodes)
                    tf.summary.scalar("training/grad_steps", grad_step, step=total_episodes)

            # ---- Periodic save ----
            if total_episodes % args.save_every < args.workers:
                _atomic_save(challenger, args.challenger_dir)
                try:
                    buffer.save(buffer_path)
                except Exception as e:
                    print(f"[challenger] Buffer save failed: {e}")
                dt = time.time() - t0
                print(
                    f"[challenger] ep={total_episodes:5d}  "
                    f"az_sims={az_sims_now:3d}  "
                    f"ply={np.mean(recent_lengths):.1f}  "
                    f"buf={len(buffer):6d}  "
                    f"W/L/D={ch_wins}/{ch_losses}/{draws}  "
                    f"wr(100)={win_rate:.1%}  "
                    f"loss={ep_total:.3f} (p={ep_pol:.3f} v={ep_val:.3f})  "
                    f"dt={dt:.0f}s"
                )

            # ---- Periodic evaluation tournament ----
            if total_episodes % args.eval_every < args.workers:
                print(f"\n[eval] Running {args.eval_games}-game tournament "
                      f"(sims={args.eval_sims}) ...")
                eval_ch_weights = challenger.get_weights()
                eval_args_list = [
                    {
                        "challenger_weights": eval_ch_weights,
                        "az_weights":         az_weights_snapshot,
                        # Alternate sides across eval games for fairness
                        "challenger_side":    "bot" if i % 2 == 0 else "player",
                        "filters":            args.filters,
                        "blocks":             args.blocks,
                        "ch_sims":            args.eval_sims,
                        "az_sims":            args.eval_sims,
                        "c_puct":             args.c_puct,
                        "dirichlet_alpha":    args.dirichlet_alpha,
                        "dirichlet_eps":      0.0,
                        "temp_moves":         0,    # greedy throughout
                        "max_plies":          args.max_plies,
                        "eval_mode":          True,
                    }
                    for i in range(args.eval_games)
                ]

                eval_results = pool.map(_game_worker, eval_args_list)

                eval_wins = eval_losses = eval_draws = 0
                for res in eval_results:
                    status   = res["status"]
                    ch_side_ = res["challenger_side"]
                    if status == "bot_wins":
                        outcome = 1.0 if ch_side_ == "bot" else -1.0
                    elif status == "player_wins":
                        outcome = 1.0 if ch_side_ == "player" else -1.0
                    else:
                        outcome = 0.0
                    if outcome > 0:
                        eval_wins += 1
                    elif outcome < 0:
                        eval_losses += 1
                    else:
                        eval_draws += 1

                eval_decisive = eval_wins + eval_losses
                eval_wr = eval_wins / eval_decisive if eval_decisive > 0 else 0.0
                print(
                    f"[eval] ep={total_episodes}  "
                    f"W/L/D={eval_wins}/{eval_losses}/{eval_draws}  "
                    f"win_rate={eval_wr:.1%}"
                )

                with writer.as_default():
                    tf.summary.scalar("eval/win_rate",   eval_wr,     step=total_episodes)
                    tf.summary.scalar("eval/wins",       eval_wins,   step=total_episodes)
                    tf.summary.scalar("eval/losses",     eval_losses, step=total_episodes)
                    tf.summary.scalar("eval/draws",      eval_draws,  step=total_episodes)

                # Save a separate "best" checkpoint whenever win rate improves
                if eval_wr > best_win_rate:
                    best_win_rate = eval_wr
                    best_dir = str(Path(args.challenger_dir).parent / "best")
                    _atomic_save(challenger, best_dir)
                    print(f"[eval] ★ New best win rate {eval_wr:.1%} → saved to {best_dir}")

    finally:
        pool.close()
        pool.join()

    # Final save
    _atomic_save(challenger, args.challenger_dir)
    try:
        buffer.save(buffer_path)
    except Exception:
        pass
    writer.close()
    print(f"\n[challenger] Done. Final weights → {args.challenger_dir}")
    print(f"[challenger] Best win rate achieved: {best_win_rate:.1%}")
    print(f"[challenger] Best weights → {Path(args.challenger_dir).parent / 'best'}")


if __name__ == "__main__":
    # CRITICAL for TensorFlow + multiprocessing on Linux
    mp.set_start_method("spawn", force=True)
    main()

