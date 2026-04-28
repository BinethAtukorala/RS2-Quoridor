"""Fixed-capacity circular buffer of AlphaZero training samples.

Stores parallel arrays of (state, policy, value) so sampling allocates
nothing. Pickle save/load lets a training run survive a restart.
"""
from __future__ import annotations

import os
import pickle
from pathlib import Path

import numpy as np

try:
    from .encoder import BOARD_N, NUM_ACTIONS, STATE_CHANNELS
except ImportError:
    from encoder import BOARD_N, NUM_ACTIONS, STATE_CHANNELS


class ReplayBuffer:
    def __init__(self, capacity: int = 100_000):
        self.capacity = capacity
        self.size = 0
        self.ptr = 0
        self.states   = np.zeros((capacity, BOARD_N, BOARD_N, STATE_CHANNELS), dtype=np.float32)
        self.policies = np.zeros((capacity, NUM_ACTIONS), dtype=np.float32)
        self.values   = np.zeros((capacity,), dtype=np.float32)

    def add(self, s: np.ndarray, pi: np.ndarray, z: float):
        i = self.ptr
        self.states[i]   = s
        self.policies[i] = pi
        self.values[i]   = z
        self.ptr = (i + 1) % self.capacity
        self.size = min(self.size + 1, self.capacity)

    def sample(self, batch_size: int):
        idx = np.random.randint(0, self.size, size=batch_size)
        return self.states[idx], self.policies[idx], self.values[idx]

    def __len__(self):
        return self.size

    def save(self, path: str):
        Path(os.path.dirname(path)).mkdir(parents=True, exist_ok=True)
        with open(path, "wb") as f:
            pickle.dump(dict(
                capacity=self.capacity, size=self.size, ptr=self.ptr,
                states=self.states[: self.size],
                policies=self.policies[: self.size],
                values=self.values[: self.size],
            ), f, protocol=pickle.HIGHEST_PROTOCOL)

    def load(self, path: str):
        with open(path, "rb") as f:
            d = pickle.load(f)
        self.capacity = d["capacity"]
        self.size = d["size"]
        self.ptr = d["ptr"] % self.capacity
        n = d["size"]
        self.states[:n]   = d["states"]
        self.policies[:n] = d["policies"]
        self.values[:n]   = d["values"]
