"""Drive a full Quoridor game between two policies and return the
resulting list of (s, a, r, s', done, mask) transitions.

Used by the offline training loops (train.py, train_vs_model.py) to
generate experience for the replay buffer.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

import numpy as np

# from quoridor_game.quoridor_utils import QuoridorBoard

try:
    from quoridor_game.quoridor_utils import QuoridorBoard
except ImportError:
    from quoridor_utils import QuoridorBoard

# from .encoder import encode_state, legal_action_mask, action_to_move, NUM_ACTIONS

try:
    from .encoder import encode_state, legal_action_mask, action_to_move, NUM_ACTIONS
except ImportError:
    from encoder import encode_state, legal_action_mask, action_to_move, NUM_ACTIONS

try:
    from .reward import potential, shaped_step
except ImportError:
    from reward import potential, shaped_step

# A policy maps (board, side-to-move, legal-action-mask) -> action index.
Policy = Callable[[QuoridorBoard, str, np.ndarray], int]


@dataclass
class Transition:
    # One DQN training sample. `next_mask` is needed so the bootstrap step
    # in the Bellman target only considers legal next actions.
    state: np.ndarray
    action: int
    reward: float
    next_state: np.ndarray
    done: bool
    next_mask: np.ndarray


def _terminal_reward(board: QuoridorBoard, side: str) -> float:
    # +1 for winning, -1 for losing, 0 for draws / no decision yet.
    if board.game_status == "bot_wins":
        return 1.0 if side == "bot" else -1.0
    if board.game_status == "player_wins":
        return 1.0 if side == "player" else -1.0
    return 0.0


def play_game(
    bot_policy: Policy,
    player_policy: Policy,
    record_sides: tuple[str, ...] = ("bot",),
    max_plies: int = 200,
    board_n: int = 5,
    shaping_coef: float = 0.0,   # 0 disables PBRS; ~0.1 is a good default
    gamma: float = 0.99,         # discount used for the PBRS term
) -> tuple[list[Transition], str]:
    board = QuoridorBoard(n=board_n)
    transitions: list[Transition] = []
    # Per-side memory: (state_tensor, action, phi_at_action_time). The
    # potential is captured at the moment the side committed to its action
    # so the shaping term gamma*Phi(s') - Phi(s) reflects the change in
    # advantage that resulted from that action.
    pending: dict[str, tuple[np.ndarray, int, float]] = {}

    ply = 0
    # max_plies guards against pathological non-terminating games during training.
    while board.game_status == "in_progress" and ply < max_plies:
        side = board.current_turn
        mask = legal_action_mask(board, side)
        if mask.sum() == 0:
            break
        state = encode_state(board, side)
        # Phi at this state (from `side`'s POV); used for both closing the
        # previous pending transition (as Phi(s')) and capturing the new
        # pending transition (as Phi(s)).
        phi_now = potential(board, side, shaping_coef) if shaping_coef > 0 else 0.0

        # Close the previous pending transition for this side (non-terminal).
        if side in record_sides and side in pending:
            prev_s, prev_a, prev_phi = pending.pop(side)
            r = 0.0
            if shaping_coef > 0:
                r = shaped_step(r, prev_phi, phi_now, gamma)
            transitions.append(Transition(prev_s, prev_a, r, state, False, mask))

        policy = bot_policy if side == "bot" else player_policy
        action = policy(board, side, mask)
        move = action_to_move(board, action, side)
        if move is None or not board.apply_move(move):
            # Illegal action: end with hard penalty (skip shaping).
            if side in record_sides:
                ns = encode_state(board, side)
                transitions.append(Transition(state, action, -1.0, ns, True,
                                              np.zeros(NUM_ACTIONS, dtype=np.float32)))
            break

        if side in record_sides:
            pending[side] = (state, action, phi_now)
        ply += 1

        if board.game_status != "in_progress":
            break

    timed_out = board.game_status == "in_progress"  # hit max_plies
    for side, (s, a, prev_phi) in pending.items():
        if timed_out:
            r = -0.1
        else:
            r = _terminal_reward(board, side)
        # Phi(terminal) = 0 by convention -> shaping subtracts prev_phi.
        if shaping_coef > 0:
            r = shaped_step(r, prev_phi, 0.0, gamma)
        ns = encode_state(board, side)
        nm = legal_action_mask(board, side) if board.game_status == "in_progress" else \
             np.zeros(NUM_ACTIONS, dtype=np.float32)
        done = board.game_status != "in_progress"
        transitions.append(Transition(s, a, r, ns, done, nm))

    return transitions, board.game_status
