"""Drive a full Quoridor game between two policies and return the
resulting list of (s, a, r, s', done, mask) transitions.

Used by the offline training loops (train.py, train_vs_model.py) to
generate experience for the replay buffer.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

import numpy as np

from quoridor_game.quoridor_utils import QuoridorBoard

from .encoder import encode_state, legal_action_mask, action_to_move, NUM_ACTIONS


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
) -> tuple[list[Transition], str]:
    board = QuoridorBoard(n=board_n)
    transitions: list[Transition] = []
    # Per-side memory: when it's our turn, we recorded (state, action) and we
    # close that transition as soon as we see the *next* state from our PoV.
    pending: dict[str, tuple[np.ndarray, int]] = {}

    ply = 0
    # max_plies guards against pathological non-terminating games during training.
    while board.game_status == "in_progress" and ply < max_plies:
        side = board.current_turn
        mask = legal_action_mask(board, side)
        if mask.sum() == 0:
            # No legal moves: treat as game over to avoid an infinite loop.
            break
        state = encode_state(board, side)

        # Close the previous pending transition for this side (non-terminal, r=0).
        if side in record_sides and side in pending:
            prev_s, prev_a = pending.pop(side)
            transitions.append(Transition(prev_s, prev_a, 0.0, state, False, mask))

        # Ask the appropriate policy for an action and try to apply it.
        policy = bot_policy if side == "bot" else player_policy
        action = policy(board, side, mask)
        move = action_to_move(board, action, side)
        if move is None or not board.apply_move(move):
            # An illegal move ends the rollout with a strongly negative
            # reward so the agent learns to avoid it.
            if side in record_sides:
                ns = encode_state(board, side)
                transitions.append(Transition(state, action, -1.0, ns, True,
                                              np.zeros(NUM_ACTIONS, dtype=np.float32)))
            break

        if side in record_sides:
            # Save what we just did so we can close it next time around.
            pending[side] = (state, action)
        ply += 1

        if board.game_status != "in_progress":
            break

    # The last action of each recorded side never had a chance to be closed
    # in the main loop -- attach the terminal reward to it here.
    for side, (s, a) in pending.items():
        r = _terminal_reward(board, side)
        ns = encode_state(board, side)
        nm = legal_action_mask(board, side) if board.game_status == "in_progress" else \
             np.zeros(NUM_ACTIONS, dtype=np.float32)
        done = board.game_status != "in_progress"
        transitions.append(Transition(s, a, r, ns, done, nm))

    return transitions, board.game_status
