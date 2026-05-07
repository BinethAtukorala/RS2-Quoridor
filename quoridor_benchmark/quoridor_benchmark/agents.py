"""Factory functions for loading benchmark agents as Policy callables.

Each returns a Policy: (board, side, mask) -> action_index.

Supported agents
----------------
- Engine (Minimax / alpha-beta) — no weights needed.
- DQN   — requires a directory with qnet.weights.h5.
- AlphaZero — requires a directory with az_net.weights.h5.
"""
from __future__ import annotations

from pathlib import Path
from typing import Callable

import numpy as np

from .quoridor_utils import QuoridorBoard
from .encoder import (
    NUM_ACTIONS,
    encode_state,
    legal_action_mask,
)
from .minimax_policy import make_minimax_policy

Policy = Callable[[QuoridorBoard, str, np.ndarray], int]


# ---------------------------------------------------------------------------
# Engine (Minimax)
# ---------------------------------------------------------------------------

def load_engine_agent(depth: int = 3, noise: float = 0.0) -> Policy:
    """Alpha-beta minimax engine.

    Parameters
    ----------
    depth : int
        Search depth (3 is strong for 5×5).
    noise : float
        Probability of random move (0 = deterministic).
    """
    return make_minimax_policy(depth=depth, opponent_noise=noise)


# ---------------------------------------------------------------------------
# DQN
# ---------------------------------------------------------------------------

def load_dqn_agent(model_dir: str, epsilon: float = 0.0) -> Policy:
    """Load a trained DQN agent from *model_dir*/qnet.weights.h5.

    Parameters
    ----------
    model_dir : str
        Directory produced by DQNAgent.save().
    epsilon : float
        Greedy epsilon at evaluation time (0 = fully greedy).
    """
    # Lazy import so TF is only pulled when needed.
    try:
        from .dqn_agent import DQNAgent
    except ImportError:
        from dqn_agent import DQNAgent

    agent = DQNAgent()
    path = Path(model_dir) / "qnet.weights.h5"
    if not path.exists():
        raise FileNotFoundError(
            f"DQN weights not found at {path}. "
            "Train the model first or point model_dir at the saved checkpoint."
        )
    agent.load(model_dir)

    def policy(board: QuoridorBoard, side: str, mask: np.ndarray) -> int:
        state = encode_state(board, side)
        return agent.select_action(state, mask, epsilon=epsilon)

    policy.__name__ = f"DQN(eps={epsilon})"
    return policy


# ---------------------------------------------------------------------------
# AlphaZero
# ---------------------------------------------------------------------------

def load_alphazero_agent(
    model_dir: str,
    n_simulations: int = 64,
    c_puct: float = 1.5,
    temperature: float = 0.0,
) -> Policy:
    """Load a trained AlphaZero agent from *model_dir*/az_net.weights.h5.

    Parameters
    ----------
    model_dir : str
        Directory produced by AlphaZero training (_atomic_save writes
        az_net.weights.h5 inside the directory).
    n_simulations : int
        MCTS simulations per move (more = stronger but slower).
    c_puct : float
        PUCT exploration constant.
    temperature : float
        Move-selection temperature. 0 → argmax (greedy).
    """
    try:
        from .az_network import build_az_net
        from .mcts import MCTS
    except ImportError:
        from az_network import build_az_net
        from mcts import MCTS

    import tensorflow as tf

    weights_path = Path(model_dir) / "az_net.weights.h5"
    if not weights_path.exists():
        raise FileNotFoundError(
            f"AlphaZero weights not found at {weights_path}. "
            "Train the model first or point model_dir at the saved checkpoint."
        )

    model = build_az_net()

    # Build the model by running a dummy forward pass, then load weights.
    dummy_input = np.zeros((1, 5, 5, 7), dtype=np.float32)
    model(dummy_input, training=False)
    model.load_weights(str(weights_path))

    def evaluator(state_batch: np.ndarray):
        logits, value = model(state_batch, training=False)
        probs = tf.nn.softmax(logits).numpy()
        return probs, value.numpy()[:, 0]

    def policy(board: QuoridorBoard, side: str, mask: np.ndarray) -> int:
        mcts = MCTS(
            evaluator=evaluator,
            n_simulations=n_simulations,
            c_puct=c_puct,
        )
        visit_counts, _ = mcts.run(board)
        legal = np.flatnonzero(mask > 0.5)
        if legal.size == 0:
            return -1
        # Temperature-based selection.
        visits = visit_counts[legal].astype(float)
        if temperature == 0.0 or visits.sum() == 0:
            return int(legal[np.argmax(visits)])
        probs = visits ** (1.0 / temperature)
        probs /= probs.sum()
        return int(np.random.choice(legal, p=probs))

    policy.__name__ = f"AlphaZero(sims={n_simulations})"
    return policy
