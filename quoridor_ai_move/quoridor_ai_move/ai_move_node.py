"""ROS 2 node that turns Quoridor board states into AI-chosen moves.

Subscribes to compute-move requests, runs the DQN agent to pick a legal
move, and publishes the chosen move back. Optionally keeps learning
online from each completed game.
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

# Quiet TF's noisy startup logs. Done before the TF import so it takes effect.
os.environ.setdefault("TF_CPP_MIN_LOG_LEVEL", "2")
import tensorflow as tf  # noqa: E402

from quoridor_game.quoridor_utils import QuoridorBoard  # noqa: E402

from .agent import DQNAgent  # noqa: E402
from .encoder import encode_state, legal_action_mask, action_to_move  # noqa: E402
from .replay_buffer import ReplayBuffer  # noqa: E402


class AiMoveNode(Node):
    def __init__(self):
        super().__init__("ai_move_node")

        # ----- ROS parameters (overridable from launch / CLI) -----
        # model_dir: where qnet.weights.h5 is loaded from / saved to.
        # online_learning: enable the experience-replay learning loop.
        # epsilon: exploration rate at inference (0 = fully greedy).
        # side: which player this node is responsible for ("bot" or "player").

        self.declare_parameter("model_dir", str(Path.home() / "quoridor_models" / "latest"))
        self.declare_parameter("online_learning", True)
        self.declare_parameter("online_lr", 1e-4)
        self.declare_parameter("save_after_game", True)
        self.declare_parameter("epsilon", 0.0)
        self.declare_parameter("filters", 64)
        self.declare_parameter("blocks", 4)
        self.declare_parameter("batch_size", 64)
        self.declare_parameter("side", "bot")  # which side this node plays

        self.model_dir = self.get_parameter("model_dir").get_parameter_value().string_value
        self.online = self.get_parameter("online_learning").get_parameter_value().bool_value
        self.online_lr = self.get_parameter("online_lr").get_parameter_value().double_value
        self.save_after_game = self.get_parameter("save_after_game").get_parameter_value().bool_value
        self.epsilon = self.get_parameter("epsilon").get_parameter_value().double_value
        self.filters = self.get_parameter("filters").get_parameter_value().integer_value
        self.blocks = self.get_parameter("blocks").get_parameter_value().integer_value
        self.batch_size = self.get_parameter("batch_size").get_parameter_value().integer_value
        self.side = self.get_parameter("side").get_parameter_value().string_value

        # Allow TF to grow GPU memory on demand instead of grabbing it all up
        # front -- important when sharing the GPU with other nodes.
        for g in tf.config.list_physical_devices("GPU"):
            try:
                tf.config.experimental.set_memory_growth(g, True)
            except Exception:
                pass

        # Build the agent and try to restore a checkpoint. A missing model is
        # not fatal -- we just warn and play with random weights.
        self.agent = DQNAgent(lr=self.online_lr, filters=self.filters, n_blocks=self.blocks)
        if self.agent.load(self.model_dir):
            self.get_logger().info(f"Loaded weights from {self.model_dir}")
        else:
            self.get_logger().warn(
                f"No weights found at {self.model_dir}; running with randomly initialized network")

        # Replay buffer for online learning during live play.
        self.buffer = ReplayBuffer(capacity=50_000)
        # Track the (state, action) chosen on our previous turn so that when
        # control returns to us we can close the transition with the resulting
        # next-state and (eventually) the terminal reward.
        self._pending_state: np.ndarray | None = None
        self._pending_action: int | None = None
        self._game_started = False
        # rclpy callbacks may run on multiple threads; serialize access to
        # the agent / buffer state.
        self._lock = threading.Lock()

        # Topic contract:
        #   in : board snapshot (JSON) on /quoridor/compute_move_request
        #   out: chosen move (JSON)    on /quoridor/compute_move_response
        self.sub = self.create_subscription(
            String, "/quoridor/compute_move_request", self.on_request, 10)
        self.pub = self.create_publisher(
            String, "/quoridor/compute_move_response", 10)

        self.get_logger().info(
            f"AI move node ready (side={self.side}, epsilon={self.epsilon}, "
            f"online_learning={self.online})")

    # ------------------------------------------------------------------ #

    def on_request(self, msg: String):
        # Single entry point for every board update we receive.
        with self._lock:
            try:
                board = QuoridorBoard.from_json(msg.data)
            except Exception as e:
                self.get_logger().error(f"Failed to parse board: {e}")
                return

            # Handle terminal state observations from upstream (e.g. state manager
            # publishing the final board): close the pending transition.
            if board.game_status != "in_progress":
                self._close_game(board)
                return

            if board.current_turn != self.side:
                # Not our turn — just observe the state so we can close a pending
                # transition when it's our turn again.
                return

            # Build the mask of legal action indices for the current position.
            mask = legal_action_mask(board, self.side)
            if mask.sum() == 0:
                self.get_logger().error("No legal moves available!")
                return

            # Encode the board into the (n, n, C) tensor the network expects.
            state = encode_state(board, self.side)

            # Close previous pending transition with this state (mid-game, r=0).
            # Reward is zero because the game hasn't ended yet; the only
            # non-zero rewards happen at terminal states (see _close_game).
            if self.online and self._pending_state is not None:
                self.buffer.add(self._pending_state, self._pending_action, 0.0,
                                state, False, mask)

            # Pick an action via the DQN policy (epsilon-greedy if epsilon > 0).
            action = self.agent.select_action(state, mask, self.epsilon)
            move = action_to_move(board, action, self.side)
            if move is None:
                # Defensive: should never happen because we masked illegal
                # actions, but fall back to any legal move rather than crash.
                self.get_logger().error(f"Invalid action index {action}; falling back to any legal move")
                action = int(np.flatnonzero(mask > 0.5)[0])
                move = action_to_move(board, action, self.side)

            if self.online:
                # Remember what we did so we can form a (s, a, r, s') tuple
                # once we observe the next state.
                self._pending_state = state
                self._pending_action = action
                self._game_started = True

            # Publish the chosen move back to whoever requested it.
            out = String()
            out.data = json.dumps(move.to_dict())
            self.pub.publish(out)
            self.get_logger().info(f"Move: {out.data}")

    # ------------------------------------------------------------------ #

    def _close_game(self, board: QuoridorBoard):
        # Called when we observe a terminal board. Emits the final transition
        # with the +/-1 reward, then runs a few SGD steps and (optionally)
        # checkpoints the model.
        if not self.online or not self._game_started:
            return
        if self._pending_state is None:
            self._game_started = False
            return

        # Map game outcome -> reward from this side's perspective.
        if board.game_status == "bot_wins":
            reward = 1.0 if self.side == "bot" else -1.0
        elif board.game_status == "player_wins":
            reward = 1.0 if self.side == "player" else -1.0
        else:
            reward = 0.0

        ns = encode_state(board, self.side)
        from .encoder import NUM_ACTIONS
        # Terminal transition: done=True, all-zero next-action mask so the
        # bootstrap term in the Bellman update is zeroed out.
        self.buffer.add(self._pending_state, self._pending_action, reward,
                        ns, True, np.zeros(NUM_ACTIONS, dtype=np.float32))

        # Reset per-game bookkeeping.
        self._pending_state = None
        self._pending_action = None
        self._game_started = False

        # Burst-train on the latest replay buffer contents. 8 steps is a
        # cheap update that fits inside the inter-game pause without making
        # the user wait noticeably.
        if len(self.buffer) >= self.batch_size:
            for _ in range(8):
                batch = self.buffer.sample(self.batch_size)
                self.agent.train_on_batch(batch)
            self.agent.update_target()
            self.get_logger().info(
                f"Online update done (buffer={len(self.buffer)}, reward={reward:+.0f})")

        # Persist weights so the next game (or process restart) picks up
        # whatever we just learned.
        if self.save_after_game:
            try:
                self.agent.save(self.model_dir)
                self.get_logger().info(f"Saved weights -> {self.model_dir}")
            except Exception as e:
                self.get_logger().warn(f"Save failed: {e}")


def main(args=None):
    # Standard rclpy boilerplate: spin until Ctrl-C, then clean up.
    rclpy.init(args=args)
    node = AiMoveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
