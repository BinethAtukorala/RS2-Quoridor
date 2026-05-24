"""Single-game match runner between two benchmark agents."""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Callable

import numpy as np

from .quoridor_utils import QuoridorBoard
from .encoder import encode_state, legal_action_mask, action_to_move, NUM_ACTIONS

# Policy signature: (board, side, mask) -> action_index
Policy = Callable[[QuoridorBoard, str, np.ndarray], int]

MAX_PLIES = 150  # hard cap to prevent infinite games


@dataclass
class MoveRecord:
    ply: int
    side: str
    action: int
    elapsed_ms: float


@dataclass
class MatchResult:
    winner: str          # "bot", "player", or "draw" (timeout)
    plies: int
    status: str          # board.game_status at end
    bot_name: str
    player_name: str
    move_times_ms: dict[str, list[float]] = field(default_factory=lambda: {"bot": [], "player": []})
    moves: list[MoveRecord] = field(default_factory=list)

    # Convenience
    def did_win(self, name: str) -> bool:
        side = "bot" if self.bot_name == name else "player"
        return self.winner == side

    @property
    def avg_move_ms(self) -> dict[str, float]:
        return {
            s: (sum(ts) / len(ts) if ts else 0.0)
            for s, ts in self.move_times_ms.items()
        }

    @property
    def max_move_ms(self) -> dict[str, float]:
        return {
            s: (max(ts) if ts else 0.0)
            for s, ts in self.move_times_ms.items()
        }


def play_match(
    bot_policy: Policy,
    player_policy: Policy,
    bot_name: str = "bot",
    player_name: str = "player",
    board_n: int = 5,
    max_plies: int = MAX_PLIES,
    seed: int | None = None,
) -> MatchResult:
    """Run one complete game and return a MatchResult."""
    if seed is not None:
        np.random.seed(seed)

    board = QuoridorBoard(n=board_n)
    # Randomise who goes first to eliminate first-mover bias.
    # The board always initialises with current_turn="player", so we
    # randomly flip it to "bot" half the time.
    if np.random.rand() < 0.5:
        board.current_turn = "bot"
    result = MatchResult(
        winner="draw",
        plies=0,
        status="timeout",
        bot_name=bot_name,
        player_name=player_name,
    )

    for ply in range(max_plies):
        side = board.current_turn
        mask = legal_action_mask(board, side)

        if mask.sum() == 0:
            break

        policy = bot_policy if side == "bot" else player_policy
        t0 = time.perf_counter()
        action = policy(board, side, mask)
        elapsed_ms = (time.perf_counter() - t0) * 1000.0

        result.move_times_ms[side].append(elapsed_ms)
        result.moves.append(MoveRecord(ply, side, action, elapsed_ms))

        move = action_to_move(board, action, side)
        if move is None or not board.apply_move(move):
            # Illegal move → forfeit
            result.winner = "player" if side == "bot" else "bot"
            result.status = "illegal_move"
            result.plies = ply + 1
            return result

        if board.game_status != "in_progress":
            break

    result.plies = len(result.moves)
    result.status = board.game_status

    if board.game_status == "bot_wins":
        result.winner = "bot"
    elif board.game_status == "player_wins":
        result.winner = "player"
    else:
        result.winner = "draw"

    return result
