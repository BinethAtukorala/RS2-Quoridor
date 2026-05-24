"""PUCT MCTS for AlphaZero.

One Node per encountered position. Children are kept lazily — only legal
actions get edge entries. The search runs N simulations from the given
root; each simulation:

  1. Walk down the tree picking a = argmax_a [Q(s,a) + c_puct * P(s,a) *
     sqrt(sum_b N(s,b)) / (1 + N(s,a))], cloning the board step-by-step.
  2. At a leaf, ask the network for (policy, value). Mask illegal actions,
     renormalize. Use value as the leaf's evaluation.
  3. Backup the (negated, two-player) value along the visited path.

Values are always from the *side-to-move-at-the-node*'s perspective: a
positive value at a child means good for the player who moves at the
child. This matches AlphaZero's two-player minimax-style backup, which is
just sign-flipping at each level.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable

import numpy as np

try:
    from quoridor_game.quoridor_utils import QuoridorBoard
except ImportError:
    from .quoridor_utils import QuoridorBoard

try:
    from .encoder import (NUM_ACTIONS, action_to_move, encode_state,
                          legal_action_mask)
except ImportError:
    from .encoder import (NUM_ACTIONS, action_to_move, encode_state,
                         legal_action_mask)


# A network evaluator: (state_tensor[B, N, N, C]) -> (policy_probs[B, A], value[B]).
# Decoupled from any specific framework so we can swap in mocks for tests.
Evaluator = Callable[[np.ndarray], tuple[np.ndarray, np.ndarray]]


@dataclass
class Node:
    side: str                      # who moves at this node ("bot"/"player")
    prior: float = 0.0             # P(s, a) from parent
    value_sum: float = 0.0         # sum of backed-up values from this node's POV
    visit_count: int = 0
    children: dict[int, "Node"] = field(default_factory=dict)
    legal_mask: np.ndarray | None = None     # cached
    is_terminal: bool = False
    terminal_value: float = 0.0    # from this node's POV; only meaningful if is_terminal

    def q(self) -> float:
        return self.value_sum / self.visit_count if self.visit_count > 0 else 0.0


def _terminal_value_for_side(board: QuoridorBoard, side: str) -> float:
    if board.game_status == "bot_wins":
        return 1.0 if side == "bot" else -1.0
    if board.game_status == "player_wins":
        return 1.0 if side == "player" else -1.0
    return 0.0  # draw / not-yet-terminal


def _other(side: str) -> str:
    return "player" if side == "bot" else "bot"


class MCTS:
    """Stateless search driver. One MCTS instance per move (cheap to create)."""

    def __init__(self,
                 evaluator: Evaluator,
                 n_simulations: int = 64,
                 c_puct: float = 1.5,
                 dirichlet_alpha: float = 0.3,
                 dirichlet_eps: float = 0.0):
        self.evaluator = evaluator
        self.n_simulations = n_simulations
        self.c_puct = c_puct
        self.dirichlet_alpha = dirichlet_alpha
        # Set dirichlet_eps > 0 only during self-play; 0 at inference.
        self.dirichlet_eps = dirichlet_eps

    # ------------------------------------------------------------------ #
    def run(self, board: QuoridorBoard) -> tuple[np.ndarray, Node]:
        """Run search from `board`. Return (visit_counts[A], root)."""
        root = Node(side=board.current_turn)
        self._expand(root, board)
        if root.is_terminal:
            counts = np.zeros(NUM_ACTIONS, dtype=np.float32)
            return counts, root

        # Mix Dirichlet noise into root priors for exploration during self-play.
        if self.dirichlet_eps > 0 and root.children:
            keys = list(root.children.keys())
            noise = np.random.dirichlet([self.dirichlet_alpha] * len(keys))
            for k, n in zip(keys, noise):
                c = root.children[k]
                c.prior = (1 - self.dirichlet_eps) * c.prior + self.dirichlet_eps * float(n)

        for _ in range(self.n_simulations):
            self._simulate(root, board.copy())

        counts = np.zeros(NUM_ACTIONS, dtype=np.float32)
        for a, c in root.children.items():
            counts[a] = c.visit_count
        return counts, root

    # ------------------------------------------------------------------ #
    def _expand(self, node: Node, board: QuoridorBoard) -> float:
        """Expand a leaf. Returns the value (from node.side's POV) to back up."""
        if board.game_status != "in_progress":
            node.is_terminal = True
            node.terminal_value = _terminal_value_for_side(board, node.side)
            return node.terminal_value

        mask = legal_action_mask(board, node.side)
        node.legal_mask = mask
        if mask.sum() == 0:
            # No legal moves -> treat as loss for side-to-move.
            node.is_terminal = True
            node.terminal_value = -1.0
            return node.terminal_value

        state = encode_state(board, node.side)
        policy, value = self.evaluator(state[None, ...])
        policy = policy[0]                          # (A,)
        value  = float(value[0])                    # scalar
        # Mask + renormalize. If the net assigns ~0 mass to all legal moves
        # (cold start), fall back to uniform-over-legal so search still
        # explores instead of stalling on whichever happened to be largest.
        policy = policy * mask
        s = policy.sum()
        if s > 1e-8:
            policy = policy / s
        else:
            policy = mask / max(1.0, mask.sum())

        for a in np.flatnonzero(mask > 0.5):
            node.children[int(a)] = Node(side=_other(node.side),
                                         prior=float(policy[a]))
        return value

    def _select_action(self, node: Node) -> int:
        # PUCT: a = argmax Q + U, U = c_puct * P * sqrt(N_parent) / (1 + N_child).
        # Q is from the *child's* POV, so flip sign for the parent's choice.
        sqrt_total = math.sqrt(max(1, sum(c.visit_count for c in node.children.values())))
        best_a, best_score = -1, -float("inf")
        for a, c in node.children.items():
            q_parent = -c.q()
            u = self.c_puct * c.prior * sqrt_total / (1 + c.visit_count)
            score = q_parent + u
            if score > best_score:
                best_score, best_a = score, a
        return best_a

    def _simulate(self, root: Node, board: QuoridorBoard):
        path: list[Node] = [root]
        node = root

        # Walk down to a leaf (a node with no children yet that hasn't been expanded).
        while node.children and not node.is_terminal:
            a = self._select_action(node)
            move = action_to_move(board, a, node.side)
            if move is None or not board.apply_move(move):
                # Defensive: PUCT-selected an action our mask said was legal
                # but apply_move rejected. Treat this branch as a loss for the
                # side-to-move and stop descending.
                child = node.children[a]
                child.is_terminal = True
                child.terminal_value = -1.0
                path.append(child)
                node = child
                break
            child = node.children[a]
            path.append(child)
            node = child

        # Evaluate / expand the leaf.
        if node.is_terminal:
            value = node.terminal_value
        else:
            value = self._expand(node, board)

        # Back up: at each step up the tree, the value seen by the parent is
        # the negation of the child's value (two-player zero-sum).
        for n in reversed(path):
            n.visit_count += 1
            n.value_sum += value
            value = -value


def visit_counts_to_policy(counts: np.ndarray, temperature: float) -> np.ndarray:
    """Convert MCTS visit counts to a probability distribution over actions.

    temperature == 0 -> deterministic argmax (greedy).
    temperature  > 0 -> proportional to counts ** (1/temperature).
    """
    if counts.sum() == 0:
        return counts.astype(np.float32)
    if temperature <= 1e-6:
        out = np.zeros_like(counts, dtype=np.float32)
        out[int(np.argmax(counts))] = 1.0
        return out
    c = counts.astype(np.float64) ** (1.0 / temperature)
    s = c.sum()
    return (c / s).astype(np.float32) if s > 0 else counts.astype(np.float32)
