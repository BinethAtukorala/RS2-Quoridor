import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from quoridor_game.quoridor_utils import (
    Move,
    MoveType,
    QuoridorBoard,
)


class MoveDecision(Node):
    """Quoridor bot engine — minimax with alpha-beta pruning.

    Subscribes to ``/quoridor/compute_move_request`` (a JSON-serialised
    ``BoardState``), computes the best move, and publishes the result on
    ``/quoridor/compute_move_response``.
    """

    DEFAULT_DEPTH = 2

    def __init__(self):
        super().__init__('move_decision')

        self.declare_parameter('search_depth', self.DEFAULT_DEPTH)
        self.max_depth = (
            self.get_parameter('search_depth').get_parameter_value().integer_value
        )

        self.sub_request = self.create_subscription(
            String, '/quoridor/compute_move_request', self.on_compute_request, 10)
        self.pub_response = self.create_publisher(
            String, '/quoridor/compute_move_response', 10)

        self.get_logger().info(
            f'Move Decision ready (search depth={self.max_depth})')

    # ------------------------------------------------------------------ #
    #  ROS2 callback                                                      #
    # ------------------------------------------------------------------ #

    def on_compute_request(self, msg: String):
        board = QuoridorBoard.from_json(msg.data)
        self.get_logger().info('Computing best move …')

        move = self.compute_best_move(board)
        if move is None:
            self.get_logger().error('No legal moves available!')
            return

        response = String()
        response.data = json.dumps(move.to_dict())
        self.pub_response.publish(response)
        self.get_logger().info(f'Best move: {response.data}')

    # ------------------------------------------------------------------ #
    #  Minimax engine                                                     #
    # ------------------------------------------------------------------ #

    def compute_best_move(self, board: QuoridorBoard) -> Move | None:
        best_score = float('-inf')
        best_move = None

        moves = self._ordered_moves(board)
        for move in moves:
            new_board = board.copy()
            new_board.apply_move(move)
            score = self._minimax(
                new_board, self.max_depth - 1, float('-inf'), float('inf'),
                maximizing=False)
            if score > best_score:
                best_score = score
                best_move = move

        return best_move

    def _minimax(self, board: QuoridorBoard, depth: int,
                 alpha: float, beta: float, maximizing: bool) -> float:
        if depth == 0 or board.game_status != "in_progress":
            return self._evaluate(board)

        moves = self._ordered_moves(board)
        if not moves:
            return self._evaluate(board)

        if maximizing:
            value = float('-inf')
            for move in moves:
                child = board.copy()
                child.apply_move(move)
                value = max(value, self._minimax(child, depth - 1, alpha, beta, False))
                alpha = max(alpha, value)
                if alpha >= beta:
                    break
            return value
        else:
            value = float('inf')
            for move in moves:
                child = board.copy()
                child.apply_move(move)
                value = min(value, self._minimax(child, depth - 1, alpha, beta, True))
                beta = min(beta, value)
                if alpha >= beta:
                    break
            return value

    # ------------------------------------------------------------------ #
    #  Evaluation                                                         #
    # ------------------------------------------------------------------ #

    def _evaluate(self, board: QuoridorBoard) -> float:
        """Heuristic: opponent's shortest path minus bot's shortest path.

        Positive values favour the bot.
        """
        if board.game_status == "bot_wins":
            return 1000.0
        if board.game_status == "player_wins":
            return -1000.0

        bot_dist = board.shortest_path_length(board.bot_pos_, board.n_ - 1)
        player_dist = board.shortest_path_length(board.player_pos_, 0)

        if bot_dist is None:
            return -500.0
        if player_dist is None:
            return 500.0

        score = float(player_dist - bot_dist)
        # Slight bonus for having more walls available
        score += 0.1 * (board.bot_walls_remaining - board.player_walls_remaining)
        return score

    # ------------------------------------------------------------------ #
    #  Move ordering (pawn moves first, then strategic walls)             #
    # ------------------------------------------------------------------ #

    def _ordered_moves(self, board: QuoridorBoard) -> list[Move]:
        pawn_moves = board.get_legal_pawn_moves()
        wall_moves = board.get_strategic_wall_placements()
        return pawn_moves + wall_moves


def main(args=None):
    rclpy.init(args=args)
    node = MoveDecision()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
