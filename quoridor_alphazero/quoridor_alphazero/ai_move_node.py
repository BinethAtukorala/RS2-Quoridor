"""ROS2 node that picks Quoridor moves with an AlphaZero-style network + MCTS.

Subscribes to /quoridor/compute_move_request, runs MCTS with the trained
policy/value net, and publishes the chosen move. Wire-compatible with
quoridor_ai_move.ai_move_node so this is a drop-in replacement.

Online learning:
  * Tracks every (position, action) the AI plays during a game.
  * On a loss, those (position, action) pairs are added to a persistent
    `loss_memory.json` so the exact same move is masked next time the
    exact same position is encountered.
  * Also fires a background `online_train` process that runs targeted
    self-play from a position late in the lost game and saves new
    weights. The node hot-reloads the weights file automatically.
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Live node runs on CPU: tiny model + batch=1 MCTS inference is faster on
# CPU than GPU (kernel-launch overhead dominates) and leaves the GPU free
# for online_train. Must be set BEFORE tensorflow is imported.
os.environ["CUDA_VISIBLE_DEVICES"] = ""
os.environ.setdefault("TF_CPP_MIN_LOG_LEVEL", "2")
import tensorflow as tf  # noqa: E402

from quoridor_game.quoridor_utils import QuoridorBoard  # noqa: E402

from .encoder import action_to_move, legal_action_mask  # noqa: E402
from .loss_memory import LossMemory, position_key  # noqa: E402
from .mcts import MCTS  # noqa: E402
from .network import build_az_net  # noqa: E402


class AzMoveNode(Node):
    def __init__(self):
        super().__init__("az_move_node")

        self.declare_parameter("model_dir", str(Path.home() / "quoridor_az_models" / "latest"))
        self.declare_parameter("filters", 32)
        self.declare_parameter("blocks", 3)
        self.declare_parameter("simulations", 200)
        self.declare_parameter("c_puct", 1.5)
        self.declare_parameter("temperature", 0.0)
        self.declare_parameter("side", "bot")
        self.declare_parameter("online_learning", True)
        self.declare_parameter("online_episodes", 40)
        self.declare_parameter("online_updates", 400)
        self.declare_parameter("critical_lookback", 6)

        self.model_dir   = self.get_parameter("model_dir").get_parameter_value().string_value
        self.filters     = self.get_parameter("filters").get_parameter_value().integer_value
        self.blocks      = self.get_parameter("blocks").get_parameter_value().integer_value
        self.simulations = self.get_parameter("simulations").get_parameter_value().integer_value
        self.c_puct      = self.get_parameter("c_puct").get_parameter_value().double_value
        self.temperature = self.get_parameter("temperature").get_parameter_value().double_value
        self.side        = self.get_parameter("side").get_parameter_value().string_value
        self.online_learning = self.get_parameter("online_learning").get_parameter_value().bool_value
        self.online_episodes = self.get_parameter("online_episodes").get_parameter_value().integer_value
        self.online_updates  = self.get_parameter("online_updates").get_parameter_value().integer_value
        self.critical_lookback = self.get_parameter("critical_lookback").get_parameter_value().integer_value

        self.model = build_az_net(filters=self.filters, n_blocks=self.blocks)
        self.weights_path = Path(self.model_dir) / "az_net.weights.h5"
        self._weights_mtime = 0.0
        self._reload_weights(initial=True)

        @tf.function(reduce_retracing=True)
        def _forward(x):
            logits, value = self.model(x, training=False)
            return tf.nn.softmax(logits, axis=-1), tf.squeeze(value, axis=-1)

        def evaluator(state_batch: np.ndarray):
            probs, value = _forward(tf.convert_to_tensor(state_batch, dtype=tf.float32))
            return probs.numpy(), value.numpy()

        self.mcts = MCTS(evaluator,
                         n_simulations=self.simulations,
                         c_puct=self.c_puct,
                         dirichlet_eps=0.0)

        self.loss_memory = LossMemory(str(Path(self.model_dir) / "loss_memory.json"))

        # Game tracking: we keep our own (position_key, action, board_json)
        # for every move we made during the current game, plus the most
        # recent observed game_status to detect terminal transitions.
        self._lock = threading.Lock()
        self._game_history: list[tuple[str, int, str]] = []
        self._last_status: str = "in_progress"
        self._training_proc: subprocess.Popen | None = None

        self.sub = self.create_subscription(
            String, "/quoridor/compute_move_request", self.on_request, 10)
        self.pub = self.create_publisher(
            String, "/quoridor/compute_move_response", 10)

        # Background watcher for hot-reload of weights file.
        self._stop_watcher = threading.Event()
        self._watcher_thread = threading.Thread(target=self._weights_watcher, daemon=True)
        self._watcher_thread.start()

        self.get_logger().info(
            f"AZ move node ready (side={self.side}, sims={self.simulations}, "
            f"temp={self.temperature}, online_learning={self.online_learning}, "
            f"loss_memory={len(self.loss_memory)})")

    # ------------------------------------------------------------------ #
    def _reload_weights(self, initial: bool = False):
        if self.weights_path.exists():
            mtime = self.weights_path.stat().st_mtime
            if mtime != self._weights_mtime:
                try:
                    self.model.load_weights(str(self.weights_path))
                    self._weights_mtime = mtime
                    if initial:
                        self.get_logger().info(f"Loaded AZ weights from {self.weights_path}")
                    else:
                        self.get_logger().info(f"Hot-reloaded AZ weights ({mtime})")
                except Exception as e:
                    self.get_logger().warn(f"Weight reload failed: {e}")
        elif initial:
            self.get_logger().warn(f"No weights at {self.weights_path}; running with random init")

    def _weights_watcher(self):
        while not self._stop_watcher.is_set():
            try:
                self._reload_weights()
            except Exception:
                pass
            time.sleep(2.0)

    # ------------------------------------------------------------------ #
    def _is_initial_board(self, board: QuoridorBoard) -> bool:
        return (len(board.walls) == 0
                and board.bot_walls_remaining == QuoridorBoard.WALLS_PER_PLAYER
                and board.player_walls_remaining == QuoridorBoard.WALLS_PER_PLAYER)

    def _on_game_end(self, board: QuoridorBoard):
        status = board.game_status
        we_won = (status == "bot_wins" and self.side == "bot") or \
                 (status == "player_wins" and self.side == "player")

        if not self._game_history:
            self.get_logger().info(f"Game ended ({status}); no AI moves recorded.")
            return

        if we_won or status not in ("bot_wins", "player_wins"):
            self.get_logger().info(f"Game ended ({status}); no learning needed.")
            return

        # Lost. Record loss memory + spawn background trainer.
        traj = [(key, act) for key, act, _ in self._game_history]
        self.loss_memory.add_loss_trajectory(traj)
        self.get_logger().info(
            f"Recorded loss: {len(traj)} (position, action) pairs added to memory "
            f"(total entries: {len(self.loss_memory)}).")

        if self.online_learning:
            self._spawn_background_training()

    def _spawn_background_training(self):
        # Don't pile up trainers if a previous one is still running.
        if self._training_proc is not None and self._training_proc.poll() is None:
            self.get_logger().info("Skipping background training: previous run still active.")
            return

        idx = max(0, len(self._game_history) - self.critical_lookback)
        critical_board_json = self._game_history[idx][2]

        cache_dir = Path(self.model_dir) / "online_cache"
        cache_dir.mkdir(parents=True, exist_ok=True)
        start_path = cache_dir / f"start_{int(time.time())}.json"
        with open(start_path, "w") as f:
            f.write(critical_board_json)

        cmd = [
            sys.executable, "-m", "quoridor_alphazero.online_train",
            "--model-dir", self.model_dir,
            "--start-board", str(start_path),
            "--filters", str(self.filters),
            "--blocks", str(self.blocks),
            "--episodes", str(self.online_episodes),
            "--simulations", str(self.simulations),
            "--updates", str(self.online_updates),
        ]
        # Hand the GPU to the trainer (the node itself is CPU-only).
        env = dict(os.environ)
        env.pop("CUDA_VISIBLE_DEVICES", None)
        try:
            self._training_proc = subprocess.Popen(
                cmd, env=env,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.get_logger().info(
                f"Spawned background trainer pid={self._training_proc.pid} "
                f"(critical position from move {idx})")
        except Exception as e:
            self.get_logger().error(f"Failed to spawn background trainer: {e}")

    # ------------------------------------------------------------------ #
    def on_request(self, msg: String):
        with self._lock:
            try:
                board = QuoridorBoard.from_json(msg.data)
            except Exception as e:
                self.get_logger().error(f"Failed to parse board: {e}")
                return

            # Reset history when a fresh game starts.
            if self._is_initial_board(board) and self._game_history:
                self.get_logger().info("New game detected; resetting history.")
                self._game_history.clear()
                self._last_status = "in_progress"

            # Game just ended.
            if board.game_status != "in_progress":
                if self._last_status == "in_progress":
                    self._on_game_end(board)
                self._last_status = board.game_status
                return

            self._last_status = board.game_status

            if board.current_turn != self.side:
                return

            mask = legal_action_mask(board, self.side)
            if mask.sum() == 0:
                self.get_logger().error("No legal moves available!")
                return

            counts, _root = self.mcts.run(board)

            # Mask known-losing actions for this exact position.
            bad = self.loss_memory.bad_actions(board, self.side)
            if bad:
                # Only mask if at least one non-bad legal move exists.
                legal_idx = set(int(i) for i in np.flatnonzero(mask > 0.5))
                survivors = legal_idx - bad
                if survivors:
                    for a in bad:
                        if 0 <= a < counts.shape[0]:
                            counts[a] = 0.0
                    self.get_logger().info(
                        f"Masked {len(bad & legal_idx)} known-losing action(s) "
                        f"at this position.")

            if counts.sum() == 0:
                action = int(np.flatnonzero(mask > 0.5)[0])
            elif self.temperature <= 1e-6:
                action = int(np.argmax(counts))
            else:
                c = counts.astype(np.float64) ** (1.0 / self.temperature)
                action = int(np.random.choice(len(c), p=c / c.sum()))

            move = action_to_move(board, action, self.side)
            if move is None:
                self.get_logger().error(f"Invalid action {action}; falling back")
                action = int(np.flatnonzero(mask > 0.5)[0])
                move = action_to_move(board, action, self.side)

            # Record what we played at this exact position.
            key = position_key(board, self.side)
            self._game_history.append((key, int(action), board.to_json()))

            out = String(); out.data = json.dumps(move.to_dict())
            self.pub.publish(out)
            self.get_logger().info(f"Move: {out.data}")

    # ------------------------------------------------------------------ #
    def destroy_node(self):
        self._stop_watcher.set()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AzMoveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
