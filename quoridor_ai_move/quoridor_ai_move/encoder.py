"""Conversions between the game's board / move objects and the flat
numpy tensors + integer action indices the neural net consumes.

The action space is a single fixed-size vector so the network can output
Q-values for every possible action regardless of board state; legality
is enforced separately with a mask.
"""
from __future__ import annotations

import numpy as np

from quoridor_game.quoridor_utils import (
    MoveType,
    Move,
    Orientation,
    Pawn,
    QuoridorBoard,
    Wall,
)

# We use a 5x5 Quoridor variant (smaller than tournament 9x9) for
# tractable training on modest hardware.
BOARD_N = 5
WALL_N = BOARD_N - 1
MAX_WALLS = QuoridorBoard.WALLS_PER_PLAYER

# All relative pawn destination offsets within a 5x5 window centered on
# the pawn (excluding "stay still"). Covers normal moves AND jumps over
# the opponent (which can land 2 squares away).
PAWN_OFFSETS: list[tuple[int, int]] = [
    (dx, dy)
    for dx in range(-2, 3)
    for dy in range(-2, 3)
    if not (dx == 0 and dy == 0)
]
NUM_PAWN_ACTIONS = len(PAWN_OFFSETS)              # 24
# Wall placements: WALL_N x WALL_N grid of slots, 2 orientations each.
NUM_WALL_ACTIONS = WALL_N * WALL_N * 2            # 32 for n=5
NUM_ACTIONS = NUM_PAWN_ACTIONS + NUM_WALL_ACTIONS  # 56
# State tensor channels:
#   0: my pawn, 1: opp pawn, 2: horizontal walls, 3: vertical walls,
#   4: my walls remaining (broadcast plane),
#   5: opp walls remaining (broadcast plane),
#   6: side-to-move flag (broadcast plane).
STATE_CHANNELS = 7


# Helpers for the "always face forward" canonicalization: when the
# player side is to move, we flip the y-axis so the agent always sees
# itself moving toward increasing y. This lets one network serve both
# sides without needing to know its color.
def _flip_y(y: int) -> int:
    return BOARD_N - 1 - y


def _flip_wall_y(wy: int) -> int:
    # horizontal wall at (wx, wy) blocks between y=wy and y=wy+1.
    # After flipping, it sits between y'=n-1-(wy+1) and y'=n-1-wy, i.e. wy' = n-2-wy.
    return WALL_N - 1 - wy


def encode_state(board: QuoridorBoard, agent_side: str) -> np.ndarray:
    """Return a (n, n, 7) float32 tensor from the agent's perspective.

    If agent_side == "player", the board is flipped along y so the agent
    always moves toward increasing y in the encoded frame.
    """
    n = board.n_
    t = np.zeros((n, n, STATE_CHANNELS), dtype=np.float32)

    # Player side gets the y-flip so "forward" is always +y.
    flip = agent_side == "player"

    # Pick "me" vs "opp" so the network always sees itself in channel 0.
    if agent_side == "bot":
        me, opp = board.bot_pos_, board.player_pos_
        my_walls, opp_walls = board.bot_walls_remaining, board.player_walls_remaining
    else:
        me, opp = board.player_pos_, board.bot_pos_
        my_walls, opp_walls = board.player_walls_remaining, board.bot_walls_remaining

    my_y = _flip_y(me.y) if flip else me.y
    opp_y = _flip_y(opp.y) if flip else opp.y
    t[me.x, my_y, 0] = 1.0
    t[opp.x, opp_y, 1] = 1.0

    # Encode each placed wall as a one-hot in channel 2 (horizontal) or 3 (vertical).
    for w in board.walls:
        wx, wy = w.pos
        if flip:
            wy = _flip_wall_y(wy)
        # Walls live on a (n-1)x(n-1) grid but we place them in an nxn tensor;
        # simply index into the top-left (n-1)x(n-1) sub-block.
        if w.orientation == Orientation.HOR:
            t[wx, wy, 2] = 1.0
        else:
            t[wx, wy, 3] = 1.0

    # Scalar features broadcast to full planes so a conv net can consume them.
    t[:, :, 4] = my_walls / float(MAX_WALLS)
    t[:, :, 5] = opp_walls / float(MAX_WALLS)
    t[:, :, 6] = 1.0 if board.current_turn == "bot" else 0.0

    return t


# ---------------------------------------------------------------------------
# Action indexing
# ---------------------------------------------------------------------------

def pawn_action_index(dx: int, dy: int) -> int:
    # Pawn moves occupy the first NUM_PAWN_ACTIONS slots of the action vector.
    return PAWN_OFFSETS.index((dx, dy))


def wall_action_index(wx: int, wy: int, orient: Orientation) -> int:
    # Wall moves come after pawn moves, with horizontal walls first then vertical.
    base = NUM_PAWN_ACTIONS + (0 if orient == Orientation.HOR else WALL_N * WALL_N)
    return base + wx * WALL_N + wy


def move_to_action(board: QuoridorBoard, move: Move, agent_side: str) -> int:
    """Map a Move object to its flat action index (in the agent's frame)."""
    flip = agent_side == "player"
    if move.move_type == MoveType.PAWN:
        # Encode as the (dx, dy) offset from the agent's current position
        # in the (possibly flipped) agent frame.
        if agent_side == "bot":
            cur = board.bot_pos_
        else:
            cur = board.player_pos_
        tx, ty = move.target.x, move.target.y
        cx, cy = cur.x, cur.y
        if flip:
            ty = _flip_y(ty)
            cy = _flip_y(cy)
        return pawn_action_index(tx - cx, ty - cy)
    else:
        w = move.wall
        wx, wy = w.pos
        orient = w.orientation
        if flip:
            wy = _flip_wall_y(wy)
            # Flipping y does not change wall orientation type (H stays H).
        return wall_action_index(wx, wy, orient)


def action_to_move(board: QuoridorBoard, action: int, agent_side: str) -> Move | None:
    """Inverse of move_to_action. Returns None on invalid index."""
    flip = agent_side == "player"
    if action < 0 or action >= NUM_ACTIONS:
        return None
    if action < NUM_PAWN_ACTIONS:
        # Pawn move: convert offset back into a real-board target square.
        dx, dy = PAWN_OFFSETS[action]
        if agent_side == "bot":
            cur = board.bot_pos_
        else:
            cur = board.player_pos_
        # dy is in agent-frame; unflip for real board
        real_dy = -dy if flip else dy
        return Move(move_type=MoveType.PAWN,
                    target=Pawn(cur.x + dx, cur.y + real_dy))
    # Wall placement: split into orientation block, then decode (wx, wy).
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
    """Return a (NUM_ACTIONS,) float32 mask: 1.0 for legal actions, 0.0 otherwise.

    Only actions that the *current turn* can legally play are considered; if
    agent_side != board.current_turn, the returned mask is all zeros.
    """
    mask = np.zeros((NUM_ACTIONS,), dtype=np.float32)
    if board.current_turn != agent_side:
        return mask
    # Enumerate every legal pawn move and wall placement and flip the
    # corresponding action bits to 1.
    for m in board.get_legal_pawn_moves():
        idx = move_to_action(board, m, agent_side)
        mask[idx] = 1.0
    for m in board.get_legal_wall_placements():
        idx = move_to_action(board, m, agent_side)
        mask[idx] = 1.0
    return mask
