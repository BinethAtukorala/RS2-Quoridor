"""State + action encoding for AlphaZero. 5x5 Quoridor variant.

Mirrors the DQN encoder (channel layout, action indexing, y-flip POV) so a
trained AZ network and a trained DQN network can be swapped at the same
ROS2 message boundary. Kept self-contained so this package builds without
depending on the DQN package.
"""
from __future__ import annotations

import numpy as np

try:
    from quoridor_game.quoridor_utils import MoveType, Move, Orientation, Pawn, QuoridorBoard, Wall
except ImportError:
    from quoridor_utils import MoveType, Move, Orientation, Pawn, QuoridorBoard, Wall

BOARD_N = 5
WALL_N = BOARD_N - 1
MAX_WALLS = QuoridorBoard.WALLS_PER_PLAYER

PAWN_OFFSETS: list[tuple[int, int]] = [
    (dx, dy) for dx in range(-2, 3) for dy in range(-2, 3) if not (dx == 0 and dy == 0)
]
NUM_PAWN_ACTIONS = len(PAWN_OFFSETS)
NUM_WALL_ACTIONS = WALL_N * WALL_N * 2
NUM_ACTIONS = NUM_PAWN_ACTIONS + NUM_WALL_ACTIONS
STATE_CHANNELS = 7

_PAWN_OFFSET_TO_IDX: dict[tuple[int, int], int] = {o: i for i, o in enumerate(PAWN_OFFSETS)}


def _flip_y(y: int) -> int:
    return BOARD_N - 1 - y


def _flip_wall_y(wy: int) -> int:
    return WALL_N - 1 - wy


def encode_state(board: QuoridorBoard, agent_side: str) -> np.ndarray:
    n = board.n_
    t = np.zeros((n, n, STATE_CHANNELS), dtype=np.float32)
    flip = agent_side == "player"

    if agent_side == "bot":
        me, opp = board.bot_pos_, board.player_pos_
        my_walls, opp_walls = board.bot_walls_remaining, board.player_walls_remaining
    else:
        me, opp = board.player_pos_, board.bot_pos_
        my_walls, opp_walls = board.player_walls_remaining, board.bot_walls_remaining

    my_y  = _flip_y(me.y) if flip else me.y
    opp_y = _flip_y(opp.y) if flip else opp.y
    t[me.x, my_y, 0]  = 1.0
    t[opp.x, opp_y, 1] = 1.0

    for w in board.walls:
        wx, wy = w.pos
        if flip:
            wy = _flip_wall_y(wy)
        if w.orientation == Orientation.HOR:
            t[wx, wy, 2] = 1.0
        else:
            t[wx, wy, 3] = 1.0

    t[:, :, 4] = my_walls  / float(MAX_WALLS)
    t[:, :, 5] = opp_walls / float(MAX_WALLS)
    t[:, :, 6] = 1.0 if board.current_turn == "bot" else 0.0
    return t


def pawn_action_index(dx: int, dy: int) -> int:
    return _PAWN_OFFSET_TO_IDX[(dx, dy)]


def wall_action_index(wx: int, wy: int, orient: Orientation) -> int:
    base = NUM_PAWN_ACTIONS + (0 if orient == Orientation.HOR else WALL_N * WALL_N)
    return base + wx * WALL_N + wy


def move_to_action(board: QuoridorBoard, move: Move, agent_side: str) -> int:
    flip = agent_side == "player"
    if move.move_type == MoveType.PAWN:
        cur = board.bot_pos_ if agent_side == "bot" else board.player_pos_
        tx, ty = move.target.x, move.target.y
        cx, cy = cur.x, cur.y
        if flip:
            ty = _flip_y(ty); cy = _flip_y(cy)
        return pawn_action_index(tx - cx, ty - cy)
    w = move.wall
    wx, wy = w.pos
    if flip:
        wy = _flip_wall_y(wy)
    return wall_action_index(wx, wy, w.orientation)


def action_to_move(board: QuoridorBoard, action: int, agent_side: str) -> Move | None:
    if action < 0 or action >= NUM_ACTIONS:
        return None
    flip = agent_side == "player"
    if action < NUM_PAWN_ACTIONS:
        dx, dy = PAWN_OFFSETS[action]
        cur = board.bot_pos_ if agent_side == "bot" else board.player_pos_
        real_dy = -dy if flip else dy
        return Move(move_type=MoveType.PAWN, target=Pawn(cur.x + dx, cur.y + real_dy))
    a = action - NUM_PAWN_ACTIONS
    if a < WALL_N * WALL_N:
        orient = Orientation.HOR
    else:
        orient = Orientation.VER
        a -= WALL_N * WALL_N
    wx, wy = divmod(a, WALL_N)
    if flip:
        wy = _flip_wall_y(wy)
    return Move(move_type=MoveType.WALL, wall=Wall(pos=(wx, wy), orientation=orient))


def legal_action_mask(board: QuoridorBoard, agent_side: str) -> np.ndarray:
    mask = np.zeros((NUM_ACTIONS,), dtype=np.float32)
    if board.current_turn != agent_side:
        return mask
    for m in board.get_legal_pawn_moves():
        mask[move_to_action(board, m, agent_side)] = 1.0
    for m in board.get_legal_wall_placements():
        mask[move_to_action(board, m, agent_side)] = 1.0
    return mask
