from __future__ import annotations

import os
import pickle
from pathlib import Path

import numpy as np

# from .encoder import BOARD_N, NUM_ACTIONS, STATE_CHANNELS

try:
    from .encoder import BOARD_N, NUM_ACTIONS, STATE_CHANNELS
except ImportError:
    from encoder import BOARD_N, NUM_ACTIONS, STATE_CHANNELS


# Fixed-capacity circular buffer holding DQN transitions
# (state, action, reward, next_state, done, next_legal_mask).
# Stored as parallel numpy arrays to keep sampling allocation-free.
class ReplayBuffer:
    def __init__(self, capacity: int = 100_000):
        self.capacity = capacity
        self.size = 0       # number of valid entries currently stored
        self.ptr = 0        # write head, wraps around at `capacity`
        # Pre-allocate every column so add()/sample() do no allocation.
        self.states = np.zeros((capacity, BOARD_N, BOARD_N, STATE_CHANNELS), dtype=np.float32)
        self.actions = np.zeros((capacity,), dtype=np.int32)
        self.rewards = np.zeros((capacity,), dtype=np.float32)
        self.next_states = np.zeros_like(self.states)
        self.dones = np.zeros((capacity,), dtype=np.float32)
        # Saving the legal mask of s' lets the agent's TD target correctly
        # ignore illegal next-actions during the max-Q lookup.
        self.next_masks = np.zeros((capacity, NUM_ACTIONS), dtype=np.float32)

    def add(self, s, a, r, s2, done, next_mask):
        # Overwrite the slot at the write head, then advance circularly.
        i = self.ptr
        self.states[i] = s
        self.actions[i] = a
        self.rewards[i] = r
        self.next_states[i] = s2
        self.dones[i] = 1.0 if done else 0.0
        self.next_masks[i] = next_mask
        self.ptr = (i + 1) % self.capacity
        self.size = min(self.size + 1, self.capacity)

    def sample(self, batch_size: int):
        # Uniform random sampling with replacement -- standard DQN behavior.
        idx = np.random.randint(0, self.size, size=batch_size)
        return (
            self.states[idx],
            self.actions[idx],
            self.rewards[idx],
            self.next_states[idx],
            self.dones[idx],
            self.next_masks[idx],
        )

    def __len__(self):
        return self.size

    # Pickle-based snapshot/restore so a long training run can survive a
    # restart without losing the accumulated experience.
    def save(self, path: str):
        Path(os.path.dirname(path)).mkdir(parents=True, exist_ok=True)
        with open(path, "wb") as f:
            pickle.dump(
                dict(
                    capacity=self.capacity,
                    size=self.size,
                    ptr=self.ptr,
                    states=self.states[: self.size],
                    actions=self.actions[: self.size],
                    rewards=self.rewards[: self.size],
                    next_states=self.next_states[: self.size],
                    dones=self.dones[: self.size],
                    next_masks=self.next_masks[: self.size],
                ),
                f,
                protocol=pickle.HIGHEST_PROTOCOL,
            )

    def load(self, path: str):
        with open(path, "rb") as f:
            d = pickle.load(f)
        self.capacity = d["capacity"]
        self.size = d["size"]
        self.ptr = d["ptr"] % self.capacity
        n = d["size"]
        self.states[:n] = d["states"]
        self.actions[:n] = d["actions"]
        self.rewards[:n] = d["rewards"]
        self.next_states[:n] = d["next_states"]
        self.dones[:n] = d["dones"]
        self.next_masks[:n] = d["next_masks"]
