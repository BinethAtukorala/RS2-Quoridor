"""ROS-driven DQN trainer that plays as the *player* against the existing
quoridor_move_decision (minimax) bot.

Expected runtime topology (start these manually before launching this node):

  ros2 run quoridor_game state_manager        # manual input mode
  ros2 run quoridor_move_decision move_decision  # the C++ minimax engine
  ros2 run quoridor_game web_interface        # optional, for visualization

This node:
  - subscribes /quoridor/board_state
  - whenever it's the player's turn, encodes the board, picks a DQN action,
    and publishes the move on /quoridor/player_move
  - records (s, a, r, s', done) transitions for the player side
  - on terminal state: closes the trajectory with +/-1 reward, runs gradient
    updates, logs everything to TensorBoard, and republishes "start" on
    /quoridor/game_command to kick off the next episode

Run:
    ros2 run quoridor_ai_move train_ros \\
        --ros-args -p model_dir:=$HOME/quoridor_models/v_ros \\
                   -p tb_log_dir:=./quoridor_tensorboard
"""
from __future__ import annotations

import json
import os
import threading
from collections import deque
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

os.environ.setdefault("TF_CPP_MIN_LOG_LEVEL", "2")
import tensorflow as tf  # noqa: E402

try:
    from quoridor_game.quoridor_utils import QuoridorBoard
except ImportError:
    from quoridor_utils import QuoridorBoard

try:
    from .agent import DQNAgent
    from .encoder import (
        NUM_ACTIONS,
        action_to_move,
        encode_state,
        legal_action_mask,
    )
    from .replay_buffer import ReplayBuffer
    from .reward import potential, shaped_step
except ImportError:
    from agent import DQNAgent
    from encoder import NUM_ACTIONS, action_to_move, encode_state, legal_action_mask
    from replay_buffer import ReplayBuffer
    from reward import potential, shaped_step


class TrainRosNode(Node):
    SIDE = "player"

    def __init__(self):
        super().__init__("train_ros")

        # ---------- params ----------
        self.declare_parameter("model_dir", str(Path.cwd() / "quoridor_models" / "v_ros"))
        self.declare_parameter("tb_log_dir", "./quoridor_tensorboard/")
        self.declare_parameter("resume", True)
        self.declare_parameter("filters", 64)
        self.declare_parameter("blocks", 4)
        self.declare_parameter("lr", 1e-3)
        self.declare_parameter("gamma", 0.99)
        self.declare_parameter("tau", 0.005)
        self.declare_parameter("batch_size", 128)
        self.declare_parameter("replay_capacity", 100_000)
        self.declare_parameter("eps_start", 1.0)
        self.declare_parameter("eps_end", 0.05)
        self.declare_parameter("eps_decay_episodes", 1500)
        self.declare_parameter("updates_per_game", 16)
        self.declare_parameter("target_sync_every", 500)
        self.declare_parameter("save_every_episodes", 25)
        self.declare_parameter("max_episodes", 0)  # 0 = unlimited
        self.declare_parameter("auto_restart", True)
        # PBRS strength. 0 disables shaping; ~0.1 gives a useful dense signal
        # without overwhelming the +-1 win/loss reward.
        self.declare_parameter("shaping_coef", 0.1)

        gp = self.get_parameter
        self.model_dir = gp("model_dir").get_parameter_value().string_value
        self.tb_log_dir = gp("tb_log_dir").get_parameter_value().string_value
        self.resume = gp("resume").get_parameter_value().bool_value
        self.filters = gp("filters").get_parameter_value().integer_value
        self.blocks = gp("blocks").get_parameter_value().integer_value
        self.lr = gp("lr").get_parameter_value().double_value
        self.gamma = gp("gamma").get_parameter_value().double_value
        self.tau = gp("tau").get_parameter_value().double_value
        self.batch_size = gp("batch_size").get_parameter_value().integer_value
        self.replay_capacity = gp("replay_capacity").get_parameter_value().integer_value
        self.eps_start = gp("eps_start").get_parameter_value().double_value
        self.eps_end = gp("eps_end").get_parameter_value().double_value
        self.eps_decay_episodes = gp("eps_decay_episodes").get_parameter_value().integer_value
        self.updates_per_game = gp("updates_per_game").get_parameter_value().integer_value
        self.target_sync_every = gp("target_sync_every").get_parameter_value().integer_value
        self.save_every = gp("save_every_episodes").get_parameter_value().integer_value
        self.max_episodes = gp("max_episodes").get_parameter_value().integer_value
        self.auto_restart = gp("auto_restart").get_parameter_value().bool_value
        self.shaping_coef = gp("shaping_coef").get_parameter_value().double_value

        # ---------- TF / agent ----------
        for g in tf.config.list_physical_devices("GPU"):
            try:
                tf.config.experimental.set_memory_growth(g, True)
            except Exception:
                pass

        self.agent = DQNAgent(lr=self.lr, gamma=self.gamma, tau=self.tau,
                              filters=self.filters, n_blocks=self.blocks)
        if self.resume and self.agent.load(self.model_dir):
            self.get_logger().info(f"Resumed weights from {self.model_dir}")
        else:
            self.get_logger().info("Starting from random weights")

        self.buffer = ReplayBuffer(capacity=self.replay_capacity)

        # ---------- TensorBoard ----------
        self.writer = tf.summary.create_file_writer(self.tb_log_dir)
        self.get_logger().info(f"TensorBoard logs -> {self.tb_log_dir}")
        self.get_logger().info(f"Run: tensorboard --logdir={self.tb_log_dir}")

        # ---------- per-episode state ----------
        self._lock = threading.Lock()
        self._pending_state: np.ndarray | None = None
        self._pending_action: int | None = None
        self._pending_phi: float = 0.0  # PBRS potential at action time
        self._acted_this_turn = False
        self._terminal_handled = False
        self._episode = 0
        self._train_step = 0
        self._wins = 0
        self._losses = 0
        self._ep_plies = 0           # plies the student played this episode
        self._ep_q_values: list[float] = []   # mean-Q telemetry
        self._recent_losses: deque[float] = deque(maxlen=50)
        self._recent_outcomes: deque[int] = deque(maxlen=50)  # 1 win, 0 loss

        # ---------- ROS I/O ----------
        self.sub_board = self.create_subscription(
            String, "/quoridor/board_state", self.on_board_state, 10)
        self.pub_player_move = self.create_publisher(
            String, "/quoridor/player_move", 10)
        self.pub_command = self.create_publisher(
            String, "/quoridor/game_command", 10)

        # Send the first "start" once everything is wired up.
        self._start_timer = self.create_timer(1.5, self._kick_off_first_game)

        self.get_logger().info(
            "TrainRosNode ready (player side). Waiting for state_manager + "
            "quoridor_move_decision to be running.")

    # ------------------------------------------------------------------ #

    def _kick_off_first_game(self):
        self._start_timer.cancel()
        self._send_start()

    def _send_start(self):
        self._episode += 1
        self._reset_episode_state()
        msg = String()
        msg.data = json.dumps({"command": "start", "bot_first": False})
        self.pub_command.publish(msg)
        self.get_logger().info(
            f"==> episode {self._episode} starting (eps={self._epsilon():.3f}, "
            f"buf={len(self.buffer)})")

    def _reset_episode_state(self):
        self._pending_state = None
        self._pending_action = None
        self._pending_phi = 0.0
        self._acted_this_turn = False
        self._terminal_handled = False
        self._ep_plies = 0
        self._ep_q_values.clear()

    def _epsilon(self) -> float:
        ep = max(0, self._episode - 1)
        return max(self.eps_end,
                   self.eps_start - (self.eps_start - self.eps_end) *
                   (ep / max(1, self.eps_decay_episodes)))

    # ------------------------------------------------------------------ #

    def on_board_state(self, msg: String):
        with self._lock:
            try:
                state_dict = json.loads(msg.data)
                board = QuoridorBoard.from_dict(state_dict)
            except Exception as e:
                self.get_logger().error(f"Bad board_state JSON: {e}")
                return

            # Terminal: train, log, restart.
            if board.game_status != "in_progress":
                if not self._terminal_handled:
                    self._terminal_handled = True
                    self._on_terminal(board)
                return

            # Not our turn or bot still moving.
            if board.current_turn != self.SIDE:
                self._acted_this_turn = False
                return
            if state_dict.get("bot_thinking") or state_dict.get("bot_executing"):
                return
            if self._acted_this_turn:
                return  # idempotency vs duplicate state messages

            mask = legal_action_mask(board, self.SIDE)
            if mask.sum() == 0:
                self.get_logger().warn("No legal moves; aborting episode")
                self._on_terminal(board)
                return

            state = encode_state(board, self.SIDE)
            phi_now = (potential(board, self.SIDE, self.shaping_coef)
                       if self.shaping_coef > 0 else 0.0)

            # Close prior pending transition (mid-game, env r=0 plus PBRS).
            if self._pending_state is not None:
                r = 0.0
                if self.shaping_coef > 0:
                    r = shaped_step(r, self._pending_phi, phi_now, self.gamma)
                self.buffer.add(self._pending_state, self._pending_action, r,
                                state, False, mask)

            # Record mean-Q for TB telemetry (greedy-policy estimate of value).
            q_vals = self.agent.q_net(state[None, ...], training=False).numpy()[0]
            q_legal = q_vals[mask > 0.5]
            if q_legal.size:
                self._ep_q_values.append(float(np.max(q_legal)))

            action = self.agent.select_action(state, mask, self._epsilon())
            move = action_to_move(board, action, self.SIDE)
            if move is None:
                action = int(np.flatnonzero(mask > 0.5)[0])
                move = action_to_move(board, action, self.SIDE)

            self._pending_state = state
            self._pending_action = action
            self._pending_phi = phi_now
            self._acted_this_turn = True
            self._ep_plies += 1

            out = String()
            out.data = json.dumps(move.to_dict())
            self.pub_player_move.publish(out)

    # ------------------------------------------------------------------ #

    def _on_terminal(self, board: QuoridorBoard):
        # Terminal reward from player's POV.
        if board.game_status == "player_wins":
            reward, outcome, won = 1.0, "WIN", 1
            self._wins += 1
        elif board.game_status == "bot_wins":
            reward, outcome, won = -1.0, "LOSS", 0
            self._losses += 1
        else:
            reward, outcome, won = 0.0, board.game_status, 0
        self._recent_outcomes.append(won)

        if self._pending_state is not None:
            ns = encode_state(board, self.SIDE)
            # Phi(terminal) = 0 -> shaping subtracts the pre-terminal potential.
            r_final = reward
            if self.shaping_coef > 0:
                r_final = shaped_step(r_final, self._pending_phi, 0.0, self.gamma)
            self.buffer.add(self._pending_state, self._pending_action, r_final,
                            ns, True, np.zeros(NUM_ACTIONS, dtype=np.float32))
            self._pending_state = None
            self._pending_action = None
            self._pending_phi = 0.0

        # Train.
        ep_loss = None
        if len(self.buffer) >= self.batch_size:
            losses = []
            for _ in range(self.updates_per_game):
                batch = self.buffer.sample(self.batch_size)
                losses.append(self.agent.train_on_batch(batch))
                self._train_step += 1
                if self._train_step % self.target_sync_every == 0:
                    self.agent.update_target()
            ep_loss = float(np.mean(losses))
            self._recent_losses.append(ep_loss)

        # TensorBoard.
        ep = self._episode
        eps = self._epsilon()
        mean_q = float(np.mean(self._ep_q_values)) if self._ep_q_values else 0.0
        rolling_wr = (sum(self._recent_outcomes) / len(self._recent_outcomes)
                      if self._recent_outcomes else 0.0)
        rolling_loss = (sum(self._recent_losses) / len(self._recent_losses)
                        if self._recent_losses else 0.0)
        with self.writer.as_default():
            tf.summary.scalar("episode/reward", reward, step=ep)
            tf.summary.scalar("episode/length", self._ep_plies, step=ep)
            tf.summary.scalar("episode/win", float(won), step=ep)
            tf.summary.scalar("episode/win_rate_50ep", rolling_wr, step=ep)
            tf.summary.scalar("episode/mean_q", mean_q, step=ep)
            if ep_loss is not None:
                tf.summary.scalar("episode/loss", ep_loss, step=ep)
                tf.summary.scalar("episode/loss_50ep", rolling_loss, step=ep)
            tf.summary.scalar("training/epsilon", eps, step=ep)
            tf.summary.scalar("training/buffer_size", len(self.buffer), step=ep)
            tf.summary.scalar("training/grad_steps", self._train_step, step=ep)
        self.writer.flush()

        # Save.
        if ep % self.save_every == 0:
            try:
                self.agent.save(self.model_dir)
                self.get_logger().info(f"Saved weights to {self.model_dir}")
            except Exception as e:
                self.get_logger().warn(f"Save failed: {e}")

        wr_total = self._wins / max(1, self._wins + self._losses)
        self.get_logger().info(
            f"<-- ep {ep} {outcome} | plies={self._ep_plies} "
            f"loss={ep_loss if ep_loss is not None else float('nan'):.4f} "
            f"meanQ={mean_q:+.3f} wr_total={wr_total:.2%} "
            f"wr_50={rolling_wr:.2%} buf={len(self.buffer)} steps={self._train_step}")

        # Next episode.
        if self.auto_restart and (self.max_episodes == 0 or ep < self.max_episodes):
            self._restart_timer = self.create_timer(0.5, self._restart_once)
        else:
            try:
                self.agent.save(self.model_dir)
            except Exception:
                pass
            self.get_logger().info("Training run complete")

    def _restart_once(self):
        if hasattr(self, "_restart_timer"):
            self._restart_timer.cancel()
        self._send_start()


def main(args=None):
    rclpy.init(args=args)
    node = TrainRosNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    try:
        node.agent.save(node.model_dir)
    except Exception:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
