import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32MultiArray
from threading import Lock
import numpy as np

from quoridor_game.quoridor_utils import (
    Move,
    MoveType,
    Orientation,
    Pawn,
    QuoridorBoard,
    Wall,
)


class StateManager(Node):

    def __init__(self):
        super().__init__('state_manager')

        self.board = QuoridorBoard()
        # Perception state: single lock guards both grids + last-applied snapshots
        self.perception_lock = Lock()
        self.latest_pawn_grid = None
        self.latest_wall_grid = None
        self.last_applied_pawn_grid = None
        self.last_applied_wall_grid = None
        self.input_mode = "manual"      # "manual" | "perception"
        self.bot_thinking = False

        # --- publishers ---
        self.pub_board_state = self.create_publisher(
            String, '/quoridor/board_state', 10)
        self.pub_compute_request = self.create_publisher(
            String, '/quoridor/compute_move_request', 10)
        # Placeholder: move execution subsystem subscribes here
        self.pub_bot_execute = self.create_publisher(
            String, '/quoridor/bot_execute', 10)

        # --- subscribers ---
        self.sub_player_move = self.create_subscription(
            String, '/quoridor/player_move', self.on_player_move, 10)
        self.sub_compute_response = self.create_subscription(
            String, '/quoridor/compute_move_response', self.on_bot_move_computed, 10)
        self.sub_game_command = self.create_subscription(
            String, '/quoridor/game_command', self.on_game_command, 10)
        
        # PERCEPTION SUBSYSTEM
        self.sub_board_perception = self.create_subscription(
            Int32MultiArray, '/perception/board_state', self.on_board_update, 10)
        self.sub_wall_perception = self.create_subscription(
            Int32MultiArray, '/perception/wall_state', self.on_wall_update, 10)

        self.get_logger().info('State Manager initialised -- waiting for "start" command')
        self.publish_board_state()

    # ------------------------------------------------------------------ #
    #  Publishing                                                         #
    # ------------------------------------------------------------------ #

    def publish_board_state(self):
        state = self.board.to_dict()
        state["input_mode"] = self.input_mode
        state["bot_thinking"] = self.bot_thinking
        msg = String()
        msg.data = json.dumps(state)
        self.pub_board_state.publish(msg)

    # ------------------------------------------------------------------ #
    #  Player move handling                                               #
    # ------------------------------------------------------------------ #

    def on_player_move(self, msg: String):
        if self.board.game_status != "in_progress":
            self.get_logger().warn('Game is not in progress -- move ignored')
            return
        if self.board.current_turn != "player":
            self.get_logger().warn('Not the player\'s turn -- move ignored')
            return

        try:
            move = Move.from_dict(json.loads(msg.data))
        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().error(f'Bad player move payload: {e}')
            return

        if not self.board.apply_move(move):
            self.get_logger().warn('Illegal player move rejected')
            return

        self.get_logger().info(f'Player move applied: {msg.data}')
        self.publish_board_state()

        if self.board.game_status != "in_progress":
            self.get_logger().info(f'Game over -- {self.board.game_status}')
            return

        self.request_bot_move()

    # ------------------------------------------------------------------ #
    #  Bot move handling                                                  #
    # ------------------------------------------------------------------ #

    def request_bot_move(self):
        self.bot_thinking = True
        self.publish_board_state()
        req = String()
        req.data = self.board.to_json()
        self.pub_compute_request.publish(req)
        self.get_logger().info('Requested bot move from Move Decision')

    def on_bot_move_computed(self, msg: String):
        self.bot_thinking = False
        try:
            move = Move.from_dict(json.loads(msg.data))
        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().error(f'Bad bot move payload: {e}')
            return

        if not self.board.apply_move(move):
            self.get_logger().error('Move Decision returned an illegal move!')
            self.publish_board_state()
            return

        self.get_logger().info(f'Bot move applied: {msg.data}')

        # Forward to move execution subsystem (placeholder)
        exec_msg = String()
        exec_msg.data = json.dumps(move.to_dict())
        self.pub_bot_execute.publish(exec_msg)

        self.publish_board_state()

        if self.board.game_status != "in_progress":
            self.get_logger().info(f'Game over -- {self.board.game_status}')

    # ------------------------------------------------------------------ #
    #  Game commands                                                      #
    # ------------------------------------------------------------------ #

    def on_game_command(self, msg: String):
        try:
            cmd = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        command = cmd.get("command", "")

        if command == "start":
            bot_first = cmd.get("bot_first", False)
            self.board = QuoridorBoard()
            self.board.current_turn = "bot" if bot_first else "player"
            self.bot_thinking = False
            self._reset_perception_snapshots()
            self.get_logger().info(
                f'New game started -- {"bot" if bot_first else "player"} goes first')
            self.publish_board_state()
            if bot_first:
                self.request_bot_move()

        elif command == "toggle_input":
            self.input_mode = (
                "perception" if self.input_mode == "manual" else "manual"
            )
            if self.input_mode == "perception":
                self._reset_perception_snapshots()
            self.get_logger().info(f'Input mode: {self.input_mode}')
            self.publish_board_state()

        elif command == "estop":
            self.board.game_status = "stopped"
            self.bot_thinking = False
            self.get_logger().warn('Emergency stop -- game halted')
            self.publish_board_state()

    # ------------------------------------------------------------------ #
    #  Perception                       #
    # ------------------------------------------------------------------ #

    def _reset_perception_snapshots(self):
        with self.perception_lock:
            self.latest_pawn_grid = None
            self.latest_wall_grid = None
            self.last_applied_pawn_grid = None
            self.last_applied_wall_grid = None

    def on_board_update(self, msg: Int32MultiArray):
        if self.input_mode != "perception":
            return
        n = self.board.n_
        data = list(msg.data)
        if len(data) != n * n:
            self.get_logger().warn(
                f'Pawn grid size mismatch: got {len(data)} entries, expected {n*n}')
            return
        grid = np.array(data, dtype=int).reshape(n, n)
        self.get_logger().info(f"Got PAWN update:\n{grid}")
        with self.perception_lock:
            self.latest_pawn_grid = grid
            self._try_apply_perception_diff_locked()

    def on_wall_update(self, msg: Int32MultiArray):
        if self.input_mode != "perception":
            return
        wn = self.board.wall_n_
        data = list(msg.data)
        if len(data) != wn * wn:
            self.get_logger().warn(
                f'Wall grid size mismatch: got {len(data)} entries, expected {wn*wn}')
            return
        grid = np.flipud(np.array(data, dtype=int).reshape(wn, wn))
        self.get_logger().info(f"Got WALL update:\n{grid}")
        with self.perception_lock:
            self.latest_wall_grid = grid
            self._try_apply_perception_diff_locked()

    def _try_apply_perception_diff_locked(self):
        """Single entry point for both perception topics. Caller must hold
        perception_lock. Looks for exactly one pawn move OR one wall placement
        vs. the last-applied snapshot and applies it."""
        if self.latest_pawn_grid is None or self.latest_wall_grid is None:
            return  # wait until we've seen at least one of each

        # First time both are available: baseline and return.
        if self.last_applied_pawn_grid is None or self.last_applied_wall_grid is None:
            self.last_applied_pawn_grid = self.latest_pawn_grid.copy()
            self.last_applied_wall_grid = self.latest_wall_grid.copy()
            return

        pawn_changed = not np.array_equal(
            self.latest_pawn_grid, self.last_applied_pawn_grid)
        wall_changed = not np.array_equal(
            self.latest_wall_grid, self.last_applied_wall_grid)

        if not pawn_changed and not wall_changed:
            return

        if pawn_changed and wall_changed:
            self.get_logger().warn(
                'Both pawn and wall grids changed simultaneously -- ignoring until stable')
            return

        move, description = (self._diff_pawn_move() if pawn_changed
                             else self._diff_wall_move())
        if move is None:
            return  # diff handler already logged why

        who = self.board.current_turn
        if not self.board.apply_move(move):
            self.get_logger().warn(
                f'Perceived {who} move ({description}) is illegal -- ignoring')
            return

        # Commit snapshot only after a successful apply.
        self.last_applied_pawn_grid = self.latest_pawn_grid.copy()
        self.last_applied_wall_grid = self.latest_wall_grid.copy()

        self.get_logger().info(f'Perception: {who} {description}')
        self.publish_board_state()

        if self.board.game_status != "in_progress":
            self.get_logger().info(f'Game over -- {self.board.game_status}')
            return

        if who == "player":
            self.request_bot_move()

    def _diff_pawn_move(self):
        past = self.last_applied_pawn_grid
        current = self.latest_pawn_grid
        disappeared, appeared = [], []
        for r in range(past.shape[0]):
            for c in range(past.shape[1]):
                p, n = int(past[r][c]), int(current[r][c])
                if p == 1 and n == 0:
                    disappeared.append((r, c))
                elif p == 0 and n == 1:
                    appeared.append((r, c))
        if len(disappeared) != 1 or len(appeared) != 1:
            self.get_logger().warn(
                f'Expected exactly one pawn move, got {len(disappeared)} disappeared '
                f'and {len(appeared)} appeared -- ignoring')
            return None, None

        from_row, from_col = disappeared[0]
        to_row, to_col = appeared[0]
        from_pos = Pawn(from_col, from_row)
        to_pos = Pawn(to_col, to_row)

        if from_pos == self.board.player_pos_:
            who = "player"
        elif from_pos == self.board.bot_pos_:
            who = "bot"
        else:
            self.get_logger().warn(
                f'Moved pawn at ({from_pos.x},{from_pos.y}) does not match '
                f'player ({self.board.player_pos_.x},{self.board.player_pos_.y}) '
                f'or bot ({self.board.bot_pos_.x},{self.board.bot_pos_.y}) -- ignoring')
            return None, None

        if who != self.board.current_turn:
            self.get_logger().warn(
                f'Detected {who} pawn move but it is {self.board.current_turn}\'s turn -- ignoring')
            return None, None

        move = Move(move_type=MoveType.PAWN, target=to_pos)
        desc = f'pawn moved from ({from_pos.x},{from_pos.y}) to ({to_pos.x},{to_pos.y})'
        return move, desc

    def _diff_wall_move(self):
        past = self.last_applied_wall_grid
        current = self.latest_wall_grid
        appeared, removed = [], []
        for r in range(past.shape[0]):
            for c in range(past.shape[1]):
                p, n = int(past[r][c]), int(current[r][c])
                if p == 0 and n != 0:
                    appeared.append((c, r, n))
                elif p != 0 and n == 0:
                    removed.append((c, r, p))
        if removed:
            self.get_logger().warn(
                f'Wall(s) disappeared from perception: {removed} -- ignoring')
            return None, None
        if len(appeared) != 1:
            self.get_logger().warn(
                f'Expected exactly one new wall, got {len(appeared)} -- ignoring')
            return None, None

        wx, wy, val = appeared[0]
        if val == 1:
            orient = Orientation.VER
        elif val == 2:
            orient = Orientation.HOR
        else:
            self.get_logger().warn(f'Unknown wall value {val} -- ignoring')
            return None, None

        move = Move(move_type=MoveType.WALL,
                    wall=Wall(pos=(wx, wy), orientation=orient))
        desc = f'placed {orient.name} wall at ({wx},{wy})'
        return move, desc


def main(args=None):
    rclpy.init(args=args)
    node = StateManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()