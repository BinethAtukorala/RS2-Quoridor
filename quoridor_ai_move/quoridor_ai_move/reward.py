"""Potential-based reward shaping (PBRS) for Quoridor.

Uses the same features as the minimax engine's evaluation in
quoridor_game.move_decision._evaluate:
    Phi_bot(s)    =  (player_dist - bot_dist) + 0.1 * (bot_walls - opp_walls)
    Phi_player(s) =  (bot_dist - player_dist) + 0.1 * (player_walls - bot_walls)

Per-step shaping added to env reward:
    F(s, a, s') = gamma * Phi(s') - Phi(s)

By the Ng-Harada-Russell theorem this is policy-invariant: the optimal
policy under shaped rewards is the same as under the original sparse +-1
rewards, so the agent still learns to *win*, just with a denser gradient
signal that propagates from the very first move.

Phi(terminal) = 0 by convention, so the win/loss signal stays intact and
PBRS just subtracts the pre-terminal potential -- this credits the agent
for the strategic advantage it built up to that point.
"""
from __future__ import annotations

try:
    from quoridor_game.quoridor_utils import QuoridorBoard
except ImportError:
    from quoridor_utils import QuoridorBoard

# Default shaping strength. With coef=0.1 and a 5x5 board, |Phi| <= 0.44, so
# per-step shaped reward stays under +-1 (i.e. comparable to terminal
# magnitude without overwhelming it).
DEFAULT_COEF = 0.1
WALL_WEIGHT = 0.1  # mirrors move_decision._evaluate's 0.1 wall coefficient


def potential(board: QuoridorBoard, side: str, coef: float = DEFAULT_COEF) -> float:
    """Side-relative potential. Positive when *side* is winning.

    Returns 0 at terminal so PBRS leaves the win/loss reward unchanged.
    """
    if board.game_status != "in_progress":
        return 0.0
    bot_dist = board.shortest_path_length(board.bot_pos_, board.n_ - 1)
    player_dist = board.shortest_path_length(board.player_pos_, 0)
    if bot_dist is None or player_dist is None:
        # Should be unreachable on legal boards, but be safe.
        return 0.0
    if side == "bot":
        d_diff = float(player_dist - bot_dist)
        w_diff = float(board.bot_walls_remaining - board.player_walls_remaining)
    else:
        d_diff = float(bot_dist - player_dist)
        w_diff = float(board.player_walls_remaining - board.bot_walls_remaining)
    return coef * (d_diff + WALL_WEIGHT * w_diff)


def shaped_step(env_reward: float, phi_prev: float, phi_next: float,
                gamma: float = 0.99) -> float:
    """Apply PBRS: r' = r + gamma*Phi(s') - Phi(s)."""
    return env_reward + gamma * phi_next - phi_prev
