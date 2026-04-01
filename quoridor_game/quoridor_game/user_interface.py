import json
import sys
import threading

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


HELP_TEXT = """
Commands:
  start              Start / restart a game (player first)
  start bot          Start a game with bot going first
  move <x> <y>       Move your pawn to column x, row y
  wall <x> <y> h     Place a horizontal wall at intersection (x, y)
  wall <x> <y> v     Place a vertical wall at intersection (x, y)
  toggle             Switch between manual / perception input
  estop              Emergency stop
  help               Show this message
  quit               Exit
""".strip()


class UserInterface(Node):

    def __init__(self):
        super().__init__('user_interface')

        self.board: QuoridorBoard | None = None
        self.input_mode = "manual"
        self.bot_thinking = False

        # --- publishers ---
        self.pub_player_move = self.create_publisher(
            String, '/quoridor/player_move', 10)
        self.pub_game_command = self.create_publisher(
            String, '/quoridor/game_command', 10)

        # --- subscribers ---
        self.sub_board_state = self.create_subscription(
            String, '/quoridor/board_state', self.on_board_state, 10)

        # Terminal input runs in a daemon thread so rclpy.spin() isn't blocked
        self._input_thread = threading.Thread(target=self._input_loop, daemon=True)
        self._input_thread.start()

        self.get_logger().info('User Interface ready')
        print('\n' + HELP_TEXT + '\n')

    # ------------------------------------------------------------------ #
    #  Board state callback                                               #
    # ------------------------------------------------------------------ #

    def on_board_state(self, msg: String):
        try:
            state = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        self.board = QuoridorBoard.from_dict(state)
        self.input_mode = state.get("input_mode", "manual")
        self.bot_thinking = state.get("bot_thinking", False)
        self._render()

    # ------------------------------------------------------------------ #
    #  Terminal rendering                                                  #
    # ------------------------------------------------------------------ #

    def _render(self):
        if self.board is None:
            return

        # Clear screen
        print("\033[2J\033[H", end="")
        print(self.board.display())
        print()

        status = self.board.game_status
        turn = self.board.current_turn
        bw = self.board.bot_walls_remaining
        pw = self.board.player_walls_remaining

        print(f"Status: {status}  |  Turn: {turn}  |  "
              f"Bot walls: {bw}  |  Player walls: {pw}  |  "
              f"Input: {self.input_mode}")

        if status != "in_progress":
            print(f"\n*** {status.replace('_', ' ').upper()} ***\n")
            return

        if self.bot_thinking:
            print("\nBot is thinking …")
        elif turn == "player" and self.input_mode == "manual":
            print("\nYour move (type 'help' for commands):")

    # ------------------------------------------------------------------ #
    #  Terminal input loop                                                #
    # ------------------------------------------------------------------ #

    def _input_loop(self):
        while rclpy.ok():
            try:
                line = input()
            except EOFError:
                break
            self._process_input(line.strip())

    def _process_input(self, line: str):
        if not line:
            return
        parts = line.split()
        cmd = parts[0].lower()

        if cmd == "help":
            print(HELP_TEXT)

        elif cmd == "quit":
            print("Exiting …")
            rclpy.shutdown()

        elif cmd == "start":
            bot_first = len(parts) > 1 and parts[1].lower() == "bot"
            self._send_command({"command": "start", "bot_first": bot_first})

        elif cmd == "toggle":
            self._send_command({"command": "toggle_input"})

        elif cmd == "estop":
            self._send_command({"command": "estop"})

        elif cmd == "move":
            if len(parts) != 3:
                print("Usage: move <x> <y>")
                return
            try:
                x, y = int(parts[1]), int(parts[2])
            except ValueError:
                print("Coordinates must be integers")
                return
            move = Move(move_type=MoveType.PAWN, target=Pawn(x, y))
            self._send_move(move)

        elif cmd == "wall":
            if len(parts) != 4:
                print("Usage: wall <x> <y> <h|v>")
                return
            try:
                x, y = int(parts[1]), int(parts[2])
            except ValueError:
                print("Coordinates must be integers")
                return
            orient_char = parts[3].lower()
            if orient_char not in ("h", "v"):
                print("Orientation must be 'h' or 'v'")
                return
            orient = Orientation.HOR if orient_char == "h" else Orientation.VER
            move = Move(move_type=MoveType.WALL, wall=Wall(pos=(x, y), orientation=orient))
            self._send_move(move)

        else:
            print(f"Unknown command: {cmd} — type 'help' for usage")

    # ------------------------------------------------------------------ #
    #  Publishing helpers                                                 #
    # ------------------------------------------------------------------ #

    def _send_move(self, move: Move):
        msg = String()
        msg.data = json.dumps(move.to_dict())
        self.pub_player_move.publish(msg)

    def _send_command(self, payload: dict):
        msg = String()
        msg.data = json.dumps(payload)
        self.pub_game_command.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UserInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
