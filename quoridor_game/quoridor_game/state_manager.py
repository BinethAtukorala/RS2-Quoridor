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

        self.board = QuoridorBoard(walls=2)
        self.board_state = {
            "current":  None,
            "past": None,
            "lock": Lock()
        }
        self.wall_state = {
            "current": None,
            "past": None,
            "lock": Lock()
        }
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
            Int32MultiArray, '/perception/'
        )

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
            self.get_logger().info(
                f'New game started -- {"bot" if bot_first else "player"} goes first')
            self.publish_board_state()
            if bot_first:
                self.request_bot_move()

        elif command == "toggle_input":
            self.input_mode = (
                "perception" if self.input_mode == "manual" else "manual"
            )
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

    def on_board_update(self, msg: Int32MultiArray):
        if self.input_mode != "perception":
            return

        data = msg.data
        board_size = self.board.n_
        board_state = []
        for i in range(board_size):
            board_state.append(data[i*board_size : i*board_size+board_size])

        # Flip vertically
        board_state = np.flipud(board_state)

        with self.board_state["lock"]:

            # First update: store as past and return
            if self.board_state["past"] is None:
                self.board_state["past"] = board_state
                return

            self.board_state["past"] = self.board_state["current"]
            self.board_state["current"] = board_state

            # No previous current to compare against yet
            if self.board_state["past"] is None:
                return

            if np.array_equal(self.board_state["past"], self.board_state["current"]):
                return

            self.get_logger().info("Board state changed -- detecting pawn move")

            # Find cells that changed: disappeared (1->0) and appeared (0->1)
            past = self.board_state["past"]
            current = self.board_state["current"]
            disappeared = []  # positions where a pawn was removed
            appeared = []     # positions where a pawn appeared

            for r in range(board_size):
                for c in range(board_size):
                    if past[r][c] == 1 and current[r][c] == 0:
                        disappeared.append((r, c))
                    elif past[r][c] == 0 and current[r][c] == 1:
                        appeared.append((r, c))

            if len(disappeared) != 1 or len(appeared) != 1:
                self.get_logger().warn(
                    f'Expected exactly one pawn move, got {len(disappeared)} disappeared '
                    f'and {len(appeared)} appeared -- ignoring')
                return

            # Grid is row, col where row=0 is top. Convert to board coords (x=col, y=row).
            from_row, from_col = disappeared[0]
            to_row, to_col = appeared[0]
            from_pos = Pawn(from_col, from_row)
            to_pos = Pawn(to_col, to_row)

            # Determine which pawn moved by matching the source position
            if from_pos == self.board.player_pos_:
                who = "player"
            elif from_pos == self.board.bot_pos_:
                who = "bot"
            else:
                self.get_logger().warn(
                    f'Moved pawn at ({from_pos.x},{from_pos.y}) does not match '
                    f'player ({self.board.player_pos_.x},{self.board.player_pos_.y}) '
                    f'or bot ({self.board.bot_pos_.x},{self.board.bot_pos_.y}) -- ignoring')
                return

            if who != self.board.current_turn:
                self.get_logger().warn(
                    f'Detected {who} pawn move but it is {self.board.current_turn}\'s turn -- ignoring')
                return

            move = Move(move_type=MoveType.PAWN, target=to_pos)

            if not self.board.apply_move(move):
                self.get_logger().warn(
                    f'Perceived {who} move to ({to_pos.x},{to_pos.y}) is illegal -- ignoring')
                return

            self.get_logger().info(
                f'Perception: {who} pawn moved from ({from_pos.x},{from_pos.y}) '
                f'to ({to_pos.x},{to_pos.y})')
            self.publish_board_state()

            if self.board.game_status != "in_progress":
                self.get_logger().info(f'Game over -- {self.board.game_status}')
                return

            # If the player just moved, trigger bot's turn
            if who == "player":
                self.request_bot_move()



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