import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

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
        # Placeholder: perception subsystem publishes here
        self.sub_perception = self.create_subscription(
            String, '/perception/board_update', self.on_perception_update, 10)

        self.get_logger().info('State Manager initialised — waiting for "start" command')
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
            self.get_logger().warn('Game is not in progress — move ignored')
            return
        if self.board.current_turn != "player":
            self.get_logger().warn('Not the player\'s turn — move ignored')
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
            self.get_logger().info(f'Game over — {self.board.game_status}')
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
            self.get_logger().info(f'Game over — {self.board.game_status}')

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
                f'New game started — {"bot" if bot_first else "player"} goes first')
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
            self.get_logger().warn('Emergency stop — game halted')
            self.publish_board_state()

    # ------------------------------------------------------------------ #
    #  Perception (placeholder)                                           #
    # ------------------------------------------------------------------ #

    def on_perception_update(self, msg: String):
        """Placeholder callback for the perception subsystem.

        When the perception package is ready it will publish detected pawn
        and wall positions here.  The State Manager should diff the perceived
        board against its internal state to infer the player's move and then
        validate / apply it.
        """
        if self.input_mode != "perception":
            return
        if self.board.current_turn != "player":
            return

        self.get_logger().info('Perception board update received (not yet implemented)')
        # TODO: diff perceived state vs self.board to extract the player move,
        #       validate it, and apply it (same flow as on_player_move).


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
