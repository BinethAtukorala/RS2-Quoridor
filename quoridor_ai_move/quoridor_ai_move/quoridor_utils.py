# quoridor_utils.py
"""
Standalone replacement for quoridor_game.quoridor_utils.
No ROS required — just plain Python dataclasses.
"""
from __future__ import annotations
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional
import math


class MoveType(Enum):
    PAWN = "pawn"
    WALL = "wall"


class Orientation(Enum):
    HOR = "horizontal"
    VER = "vertical"


@dataclass
class Pawn:
    x: int
    y: int


@dataclass
class Wall:
    pos: tuple[int, int]   # (wx, wy) top-left anchor on the wall grid
    orientation: Orientation


@dataclass
class Move:
    move_type: MoveType
    target: Optional[Pawn] = None   # used when move_type == PAWN
    wall: Optional[Wall] = None     # used when move_type == WALL

    def to_dict(self):
        if self.move_type == MoveType.PAWN:
            return {"type": "pawn", "x": self.target.x, "y": self.target.y}
        else:
            return {
                "type": "wall",
                "wx": self.wall.pos[0],
                "wy": self.wall.pos[1],
                "orientation": self.wall.orientation.value,
            }


class QuoridorBoard:
    WALLS_PER_PLAYER = 4   # 5x5 variant

    def __init__(self, n: int = 5):
        self.n_ = n
        # Pawns start on opposite sides, centred
        mid = n // 2
        self.bot_pos_ = Pawn(mid, 0)          # bot starts at bottom row
        self.player_pos_ = Pawn(mid, n - 1)   # player starts at top row
        self.bot_walls_remaining = self.WALLS_PER_PLAYER
        self.player_walls_remaining = self.WALLS_PER_PLAYER
        self.walls: list[Wall] = []
        self.current_turn = "bot"
        self.game_status = "in_progress"  # "in_progress" | "bot_wins" | "player_wins"

    # ------------------------------------------------------------------
    # Move application
    # ------------------------------------------------------------------

    def apply_move(self, move: Move) -> bool:
        """Apply a move. Returns True if legal and applied, False otherwise."""
        if self.game_status != "in_progress":
            return False

        if move.move_type == MoveType.PAWN:
            ok = self._apply_pawn(move)
        else:
            ok = self._apply_wall(move)

        if ok:
            # Check win condition
            self._check_win()
            # Swap turn
            if self.game_status == "in_progress":
                self.current_turn = "player" if self.current_turn == "bot" else "bot"
        return ok

    def _apply_pawn(self, move: Move) -> bool:
        tx, ty = move.target.x, move.target.y
        if not (0 <= tx < self.n_ and 0 <= ty < self.n_):
            return False
        legal = self.get_legal_pawn_moves()
        targets = {(m.target.x, m.target.y) for m in legal}
        if (tx, ty) not in targets:
            return False
        if self.current_turn == "bot":
            self.bot_pos_ = Pawn(tx, ty)
        else:
            self.player_pos_ = Pawn(tx, ty)
        return True

    def _apply_wall(self, move: Move) -> bool:
        # Check wall budget
        if self.current_turn == "bot" and self.bot_walls_remaining <= 0:
            return False
        if self.current_turn == "player" and self.player_walls_remaining <= 0:
            return False
        legal = self.get_legal_wall_placements()
        legal_set = {(m.wall.pos, m.wall.orientation) for m in legal}
        if (move.wall.pos, move.wall.orientation) not in legal_set:
            return False
        self.walls.append(move.wall)
        if self.current_turn == "bot":
            self.bot_walls_remaining -= 1
        else:
            self.player_walls_remaining -= 1
        return True

    # ------------------------------------------------------------------
    # Win detection
    # ------------------------------------------------------------------

    def _check_win(self):
        # Bot wins by reaching the far row (y = n-1)
        if self.bot_pos_.y == self.n_ - 1:
            self.game_status = "bot_wins"
        # Player wins by reaching row 0
        elif self.player_pos_.y == 0:
            self.game_status = "player_wins"

    # ------------------------------------------------------------------
    # Legal move generation
    # ------------------------------------------------------------------

    def get_legal_pawn_moves(self) -> list[Move]:
        """Return all legal pawn moves for the current player."""
        if self.current_turn == "bot":
            cur = self.bot_pos_
            opp = self.player_pos_
        else:
            cur = self.player_pos_
            opp = self.bot_pos_

        moves = []
        for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            nx, ny = cur.x + dx, cur.y + dy
            if not (0 <= nx < self.n_ and 0 <= ny < self.n_):
                continue
            if self._wall_blocks(cur.x, cur.y, nx, ny):
                continue
            # If occupied by opponent, try to jump
            if nx == opp.x and ny == opp.y:
                jx, jy = nx + dx, ny + dy
                if (0 <= jx < self.n_ and 0 <= jy < self.n_
                        and not self._wall_blocks(nx, ny, jx, jy)):
                    moves.append(Move(MoveType.PAWN, target=Pawn(jx, jy)))
                else:
                    # Try diagonal jumps
                    for sdx, sdy in [(dx, 0), (0, dy)] if dx != 0 else [(1, 0), (-1, 0)]:
                        if sdx == dx and sdy == dy:
                            continue
                        sx, sy = nx + sdx, ny + sdy
                        if (0 <= sx < self.n_ and 0 <= sy < self.n_
                                and not self._wall_blocks(nx, ny, sx, sy)):
                            moves.append(Move(MoveType.PAWN, target=Pawn(sx, sy)))
            else:
                moves.append(Move(MoveType.PAWN, target=Pawn(nx, ny)))
        return moves

    def get_legal_wall_placements(self) -> list[Move]:
        """Return all legal wall placements for the current player."""
        walls = []
        remaining = (self.bot_walls_remaining if self.current_turn == "bot"
                     else self.player_walls_remaining)
        if remaining <= 0:
            return walls

        wall_n = self.n_ - 1
        for wx in range(wall_n):
            for wy in range(wall_n):
                for orient in (Orientation.HOR, Orientation.VER):
                    w = Wall(pos=(wx, wy), orientation=orient)
                    if not self._wall_overlaps(w) and not self._wall_blocks_all_paths(w):
                        walls.append(Move(MoveType.WALL, wall=w))
        return walls

    # ------------------------------------------------------------------
    # Wall collision helpers
    # ------------------------------------------------------------------

    def _wall_blocks(self, x1, y1, x2, y2) -> bool:
        """Does any placed wall block movement from (x1,y1) to (x2,y2)?"""
        dx, dy = x2 - x1, y2 - y1
        for w in self.walls:
            wx, wy = w.pos
            if dy == 1:   # moving up (increasing y)
                if w.orientation == Orientation.HOR:
                    if (wx == x1 and wy == y1) or (wx == x1 - 1 and wy == y1):
                        return True
            elif dy == -1:  # moving down
                if w.orientation == Orientation.HOR:
                    if (wx == x1 and wy == y1 - 1) or (wx == x1 - 1 and wy == y1 - 1):
                        return True
            elif dx == 1:   # moving right
                if w.orientation == Orientation.VER:
                    if (wx == x1 and wy == y1) or (wx == x1 and wy == y1 - 1):
                        return True
            elif dx == -1:  # moving left
                if w.orientation == Orientation.VER:
                    if (wx == x1 - 1 and wy == y1) or (wx == x1 - 1 and wy == y1 - 1):
                        return True
        return False

    def _wall_overlaps(self, new_wall: Wall) -> bool:
        """Does new_wall overlap or cross any existing wall?"""
        wx, wy = new_wall.pos
        for w in self.walls:
            ex, ey = w.pos
            if w.orientation == new_wall.orientation:
                # Same orientation: overlaps if anchors are adjacent
                if new_wall.orientation == Orientation.HOR:
                    if ey == wy and abs(ex - wx) <= 1:
                        return True
                else:
                    if ex == wx and abs(ey - wy) <= 1:
                        return True
            else:
                # Cross: HOR and VER cross at the same anchor
                if ex == wx and ey == wy:
                    return True
        return False

    def _wall_blocks_all_paths(self, new_wall: Wall) -> bool:
        """
        Returns True if placing new_wall would trap either pawn with no
        path to their goal row. Uses BFS.
        """
        # Temporarily add the wall
        self.walls.append(new_wall)
        bot_ok = self._has_path(self.bot_pos_, target_y=self.n_ - 1)
        player_ok = self._has_path(self.player_pos_, target_y=0)
        self.walls.pop()
        return not (bot_ok and player_ok)

    def _has_path(self, start: Pawn, target_y: int) -> bool:
        """BFS: can `start` reach any cell with y == target_y?"""
        visited = set()
        queue = [(start.x, start.y)]
        visited.add((start.x, start.y))
        while queue:
            x, y = queue.pop(0)
            if y == target_y:
                return True
            for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                nx, ny = x + dx, y + dy
                if (0 <= nx < self.n_ and 0 <= ny < self.n_
                        and (nx, ny) not in visited
                        and not self._wall_blocks(x, y, nx, ny)):
                    visited.add((nx, ny))
                    queue.append((nx, ny))
        return False