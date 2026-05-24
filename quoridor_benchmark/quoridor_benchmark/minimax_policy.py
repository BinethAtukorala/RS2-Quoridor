# """Plain-Python minimax engine wrapped as a play_game Policy.

# The algorithm is identical to quoridor_game.move_decision.MoveDecision
# (alpha-beta over pawn moves + strategic wall placements, with an
# opponent-distance evaluation), but lifted out of the rclpy.Node so it
# can be called directly inside a training loop without booting ROS.
# """
# from __future__ import annotations

# import numpy as np

# try:
#     from quoridor_game.quoridor_utils import Move, QuoridorBoard
# except ImportError:
#     from quoridor_utils import Move, QuoridorBoard

# try:
#     from .encoder import move_to_action
# except ImportError:
#     from encoder import move_to_action


# def _evaluate(board: QuoridorBoard) -> float:
#     if board.game_status == "bot_wins":
#         return 1000.0
#     if board.game_status == "player_wins":
#         return -1000.0
#     bot_dist = board.shortest_path_length(board.bot_pos_, board.n_ - 1)
#     player_dist = board.shortest_path_length(board.player_pos_, 0)
#     if bot_dist is None:
#         return -500.0
#     if player_dist is None:
#         return 500.0
#     score = float(player_dist - bot_dist)
#     score += 0.1 * (board.bot_walls_remaining - board.player_walls_remaining)
#     return score


# def _ordered_moves(board: QuoridorBoard) -> list[Move]:
#     return board.get_legal_pawn_moves() + board.get_strategic_wall_placements()


# def _minimax(board: QuoridorBoard, depth: int, alpha: float, beta: float,
#              maximizing: bool) -> float:
#     if depth == 0 or board.game_status != "in_progress":
#         return _evaluate(board)
#     moves = _ordered_moves(board)
#     if not moves:
#         return _evaluate(board)
#     if maximizing:
#         v = float("-inf")
#         for m in moves:
#             child = board.copy()
#             child.apply_move(m)
#             v = max(v, _minimax(child, depth - 1, alpha, beta, False))
#             alpha = max(alpha, v)
#             if alpha >= beta:
#                 break
#         return v
#     else:
#         v = float("inf")
#         for m in moves:
#             child = board.copy()
#             child.apply_move(m)
#             v = min(v, _minimax(child, depth - 1, alpha, beta, True))
#             beta = min(beta, v)
#             if alpha >= beta:
#                 break
#         return v


# def compute_best_move(board: QuoridorBoard, depth: int = 3) -> Move | None:
#     """Top-level minimax search. Plays from the side-to-move's POV.

#     The base evaluation is bot-centric (positive = good for bot); when it's
#     the player's turn we negate the search result so 'maximizing' still
#     means 'good for the side to move'.
#     """
#     sign = 1.0 if board.current_turn == "bot" else -1.0
#     best_score = float("-inf")
#     best_move: Move | None = None
#     for m in _ordered_moves(board):
#         child = board.copy()
#         child.apply_move(m)
#         score = sign * _minimax(child, depth - 1, float("-inf"), float("inf"),
#                                 maximizing=False)
#         if score > best_score:
#             best_score = score
#             best_move = m
#     return best_move


# def make_minimax_policy(depth: int = 3):
#     """Return a Policy(board, side, mask) -> action_index using minimax."""

#     def policy(board: QuoridorBoard, side: str, mask: np.ndarray) -> int:
#         move = compute_best_move(board, depth=depth)
#         if move is None:
#             legal = np.flatnonzero(mask > 0.5)
#             return int(legal[0]) if legal.size else -1
#         return move_to_action(board, move, side)

#     return policy


"""Plain-Python minimax engine wrapped as a play_game Policy.

The algorithm is identical to quoridor_game.move_decision.MoveDecision
(alpha-beta over pawn moves + strategic wall placements, with an
opponent-distance evaluation), but lifted out of the rclpy.Node so it
can be called directly inside a training loop without booting ROS.
"""
from __future__ import annotations

import numpy as np

try:
    from quoridor_game.quoridor_utils import Move, QuoridorBoard
except ImportError:
    from .quoridor_utils import Move, QuoridorBoard

try:
    from .encoder import move_to_action
except ImportError:
    from .encoder import move_to_action


def _evaluate(board: QuoridorBoard) -> float:
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
    score += 0.1 * (board.bot_walls_remaining - board.player_walls_remaining)
    return score


def _ordered_moves(board: QuoridorBoard) -> list[Move]:
    return board.get_legal_pawn_moves() + board.get_strategic_wall_placements()


def _minimax(board: QuoridorBoard, depth: int, alpha: float, beta: float,
             maximizing: bool) -> float:
    if depth == 0 or board.game_status != "in_progress":
        return _evaluate(board)
    moves = _ordered_moves(board)
    if not moves:
        return _evaluate(board)
    if maximizing:
        v = float("-inf")
        for m in moves:
            child = board.copy()
            child.apply_move(m)
            v = max(v, _minimax(child, depth - 1, alpha, beta, False))
            alpha = max(alpha, v)
            if alpha >= beta:
                break
        return v
    else:
        v = float("inf")
        for m in moves:
            child = board.copy()
            child.apply_move(m)
            v = min(v, _minimax(child, depth - 1, alpha, beta, True))
            beta = min(beta, v)
            if alpha >= beta:
                break
        return v


def compute_best_move(board: QuoridorBoard, depth: int = 3) -> Move | None:
    """Top-level minimax search. Plays from the side-to-move's POV.

    The base evaluation is bot-centric (positive = good for bot); when it's
    the player's turn we negate the search result so 'maximizing' still
    means 'good for the side to move'.
    """
    sign = 1.0 if board.current_turn == "bot" else -1.0
    best_score = float("-inf")
    best_move: Move | None = None
    for m in _ordered_moves(board):
        child = board.copy()
        child.apply_move(m)
        score = sign * _minimax(child, depth - 1, float("-inf"), float("inf"),
                                maximizing=False)
        if score > best_score:
            best_score = score
            best_move = m
    return best_move


def make_minimax_policy(depth: int = 3, opponent_noise: float = 0.1):
    """Return a Policy(board, side, mask) -> action_index using minimax.

    opponent_noise: probability [0, 1] of picking a uniformly random legal
    move instead of the minimax-best move.  Breaks the fully deterministic
    opponent loop that causes game repetition when the student's epsilon is
    low.  0.1 (10%) is a good default — strong enough to diversify games
    without making the opponent noticeably weaker.
    """

    def policy(board: QuoridorBoard, side: str, mask: np.ndarray) -> int:
        legal = np.flatnonzero(mask > 0.5)
        if legal.size == 0:
            return -1
        # Occasionally defect from best play to inject variety.
        if opponent_noise > 0.0 and np.random.random() < opponent_noise:
            return int(np.random.choice(legal))
        move = compute_best_move(board, depth=depth)
        if move is None:
            return int(legal[0])
        return move_to_action(board, move, side)

    return policy