"""ROS2 node that picks Quoridor moves with an AlphaZero-style network + MCTS.

Subscribes to /quoridor/compute_move_request, runs MCTS with the trained
policy/value net, and publishes the chosen move. Wire-compatible with
quoridor_ai_move.ai_move_node so this is a drop-in replacement.

Online learning is intentionally NOT implemented here — AlphaZero requires
batched self-play games, which doesn't fit a per-move ROS callback. Use
the train.py script for learning.
"""
from __future__ import annotations

import json
import os
import threading
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

os.environ.setdefault("TF_CPP_MIN_LOG_LEVEL", "2")
import tensorflow as tf  # noqa: E402

from quoridor_game.quoridor_utils import QuoridorBoard  # noqa: E402

from .encoder import action_to_move, legal_action_mask  # noqa: E402
from .mcts import MCTS  # noqa: E402
from .network import build_az_net  # noqa: E402


class AzMoveNode(Node):
    def __init__(self):
        super().__init__("az_move_node")

        self.declare_parameter("model_dir", str(Path.home() / "quoridor_az_models" / "latest"))
        self.declare_parameter("filters", 32)
        self.declare_parameter("blocks", 3)
        self.declare_parameter("simulations", 200)   # heavier search at inference is fine
        self.declare_parameter("c_puct", 1.5)
        self.declare_parameter("temperature", 0.0)   # 0 = greedy argmax over visit counts
        self.declare_parameter("side", "bot")

        self.model_dir   = self.get_parameter("model_dir").get_parameter_value().string_value
        self.filters     = self.get_parameter("filters").get_parameter_value().integer_value
        self.blocks      = self.get_parameter("blocks").get_parameter_value().integer_value
        self.simulations = self.get_parameter("simulations").get_parameter_value().integer_value
        self.c_puct      = self.get_parameter("c_puct").get_parameter_value().double_value
        self.temperature = self.get_parameter("temperature").get_parameter_value().double_value
        self.side        = self.get_parameter("side").get_parameter_value().string_value

        for g in tf.config.list_physical_devices("GPU"):
            try: tf.config.experimental.set_memory_growth(g, True)
            except Exception: pass

        self.model = build_az_net(filters=self.filters, n_blocks=self.blocks)
        wp = Path(self.model_dir) / "az_net.weights.h5"
        if wp.exists():
            self.model.load_weights(str(wp))
            self.get_logger().info(f"Loaded AZ weights from {wp}")
        else:
            self.get_logger().warn(f"No weights at {wp}; running with random init")

        @tf.function(reduce_retracing=True)
        def _forward(x):
            logits, value = self.model(x, training=False)
            return tf.nn.softmax(logits, axis=-1), tf.squeeze(value, axis=-1)

        def evaluator(state_batch: np.ndarray):
            probs, value = _forward(tf.convert_to_tensor(state_batch, dtype=tf.float32))
            return probs.numpy(), value.numpy()

        # Inference: no Dirichlet noise (we want the policy, not exploration).
        self.mcts = MCTS(evaluator,
                         n_simulations=self.simulations,
                         c_puct=self.c_puct,
                         dirichlet_eps=0.0)

        self._lock = threading.Lock()
        self.sub = self.create_subscription(
            String, "/quoridor/compute_move_request", self.on_request, 10)
        self.pub = self.create_publisher(
            String, "/quoridor/compute_move_response", 10)

        self.get_logger().info(
            f"AZ move node ready (side={self.side}, sims={self.simulations}, "
            f"temp={self.temperature})")

    def on_request(self, msg: String):
        with self._lock:
            try:
                board = QuoridorBoard.from_json(msg.data)
            except Exception as e:
                self.get_logger().error(f"Failed to parse board: {e}")
                return
            if board.game_status != "in_progress" or board.current_turn != self.side:
                return

            mask = legal_action_mask(board, self.side)
            if mask.sum() == 0:
                self.get_logger().error("No legal moves available!")
                return

            counts, _root = self.mcts.run(board)
            if counts.sum() == 0:
                # Fallback: random legal move.
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

            out = String(); out.data = json.dumps(move.to_dict())
            self.pub.publish(out)
            self.get_logger().info(f"Move: {out.data}")


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
