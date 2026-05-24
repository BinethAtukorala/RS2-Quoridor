"""File-backed memory of (position, action) pairs that previously led to a loss.

Used by ai_move_node to deterministically avoid playing the exact same move
at the exact same position twice in a row across sessions. Generalisation
to similar positions is handled separately by the background trainer.
"""
from __future__ import annotations

import hashlib
import json
import os
import threading
from pathlib import Path

import numpy as np

try:
    from .encoder import encode_state
except ImportError:
    from encoder import encode_state


def position_key(board, side: str) -> str:
    state = encode_state(board, side)
    return hashlib.sha1(state.tobytes()).hexdigest()


class LossMemory:
    def __init__(self, path: str):
        self.path = path
        self._lock = threading.Lock()
        self._mem: dict[str, set[int]] = self._load()

    def _load(self) -> dict[str, set[int]]:
        if not os.path.exists(self.path):
            return {}
        try:
            with open(self.path, "r") as f:
                raw = json.load(f)
            return {k: set(int(a) for a in v) for k, v in raw.items()}
        except Exception:
            return {}

    def save(self):
        Path(os.path.dirname(self.path)).mkdir(parents=True, exist_ok=True)
        tmp = self.path + ".tmp"
        with self._lock:
            data = {k: sorted(list(v)) for k, v in self._mem.items()}
        with open(tmp, "w") as f:
            json.dump(data, f)
        os.replace(tmp, self.path)

    def bad_actions(self, board, side: str) -> set[int]:
        with self._lock:
            return set(self._mem.get(position_key(board, side), ()))

    def add_loss_trajectory(self, trajectory: list[tuple[str, int]]):
        """trajectory: list of (position_key, action) for moves *we* made in
        the lost game."""
        with self._lock:
            for key, action in trajectory:
                self._mem.setdefault(key, set()).add(int(action))
        self.save()

    def __len__(self):
        with self._lock:
            return sum(len(v) for v in self._mem.values())
