"""Generate one full self-play game and return AlphaZero training samples.

Each step records (state, mcts_policy, side); the per-sample value target
z is filled in at the end of the game from the final outcome and is +1
for the side that won, -1 for the side that lost, 0 for draws / timeouts.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

try:
    from quoridor_game.quoridor_utils import QuoridorBoard
except ImportError:
    from quoridor_utils import QuoridorBoard

try:
    from .encoder import action_to_move, encode_state
    from .mcts import MCTS, visit_counts_to_policy
except ImportError:
    from encoder import action_to_move, encode_state
    from mcts import MCTS, visit_counts_to_policy


@dataclass
class Sample:
    state: np.ndarray   # (N, N, C) from `side`'s POV
    policy: np.ndarray  # (NUM_ACTIONS,) MCTS visit-count distribution
    side: str           # "bot" or "player" — used to assign z at game end
    z: float = 0.0      # filled in after the game finishes


def _outcome_value(status: str, side: str) -> float:
    if status == "bot_wins":
        return 1.0 if side == "bot" else -1.0
    if status == "player_wins":
        return 1.0 if side == "player" else -1.0
    return 0.0


def play_game(mcts: MCTS,
              max_plies: int = 75,
              board_n: int = 5,
              temp_moves: int = 12,
              temp_high: float = 1.0,
              temp_low: float = 0.0) -> tuple[list[Sample], str, int]:
    """Play one self-play game using MCTS for both sides.

    First `temp_moves` plies sample from visit counts (temperature=temp_high)
    for exploration; later plies pick the argmax (temperature=temp_low) for
    sharper, on-policy data.
    """
    board = QuoridorBoard(n=board_n)
    samples: list[Sample] = []
    ply = 0

    while board.game_status == "in_progress" and ply < max_plies:
        side = board.current_turn
        counts, _root = mcts.run(board)
        if counts.sum() == 0:
            break

        temp = temp_high if ply < temp_moves else temp_low
        policy_target = visit_counts_to_policy(counts, temperature=temp_high)
        play_dist     = visit_counts_to_policy(counts, temperature=temp)

        samples.append(Sample(
            state=encode_state(board, side),
            policy=policy_target,
            side=side,
        ))

        if temp <= 1e-6:
            action = int(np.argmax(play_dist))
        else:
            action = int(np.random.choice(len(play_dist), p=play_dist))

        move = action_to_move(board, action, side)
        if move is None or not board.apply_move(move):
            break
        ply += 1

    status = board.game_status
    for s in samples:
        s.z = _outcome_value(status, s.side)
    return samples, status, ply
