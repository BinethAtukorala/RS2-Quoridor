# # quoridor_utils.py
# """
# Standalone replacement for quoridor_game.quoridor_utils.
# No ROS required — just plain Python dataclasses.
# """
# from __future__ import annotations
# from dataclasses import dataclass, field
# from enum import Enum
# from typing import Optional
# import math


# class MoveType(Enum):
#     PAWN = "pawn"
#     WALL = "wall"


# class Orientation(Enum):
#     HOR = "horizontal"
#     VER = "vertical"


# @dataclass
# class Pawn:
#     x: int
#     y: int


# @dataclass
# class Wall:
#     pos: tuple[int, int]   # (wx, wy) top-left anchor on the wall grid
#     orientation: Orientation


# @dataclass
# class Move:
#     move_type: MoveType
#     target: Optional[Pawn] = None   # used when move_type == PAWN
#     wall: Optional[Wall] = None     # used when move_type == WALL

#     def to_dict(self):
#         if self.move_type == MoveType.PAWN:
#             return {"type": "pawn", "x": self.target.x, "y": self.target.y}
#         else:
#             return {
#                 "type": "wall",
#                 "wx": self.wall.pos[0],
#                 "wy": self.wall.pos[1],
#                 "orientation": self.wall.orientation.value,
#             }


# class QuoridorBoard:
#     WALLS_PER_PLAYER = 4   # 5x5 variant

#     def __init__(self, n: int = 5):
#         self.n_ = n
#         # Pawns start on opposite sides, centred
#         mid = n // 2
#         self.bot_pos_ = Pawn(mid, 0)          # bot starts at bottom row
#         self.player_pos_ = Pawn(mid, n - 1)   # player starts at top row
#         self.bot_walls_remaining = self.WALLS_PER_PLAYER
#         self.player_walls_remaining = self.WALLS_PER_PLAYER
#         self.walls: list[Wall] = []
#         self.current_turn = "bot"
#         self.game_status = "in_progress"  # "in_progress" | "bot_wins" | "player_wins"

#     # ------------------------------------------------------------------
#     # Move application
#     # ------------------------------------------------------------------

#     def apply_move(self, move: Move) -> bool:
#         """Apply a move. Returns True if legal and applied, False otherwise."""
#         if self.game_status != "in_progress":
#             return False

#         if move.move_type == MoveType.PAWN:
#             ok = self._apply_pawn(move)
#         else:
#             ok = self._apply_wall(move)

#         if ok:
#             # Check win condition
#             self._check_win()
#             # Swap turn
#             if self.game_status == "in_progress":
#                 self.current_turn = "player" if self.current_turn == "bot" else "bot"
#         return ok

#     def _apply_pawn(self, move: Move) -> bool:
#         tx, ty = move.target.x, move.target.y
#         if not (0 <= tx < self.n_ and 0 <= ty < self.n_):
#             return False
#         legal = self.get_legal_pawn_moves()
#         targets = {(m.target.x, m.target.y) for m in legal}
#         if (tx, ty) not in targets:
#             return False
#         if self.current_turn == "bot":
#             self.bot_pos_ = Pawn(tx, ty)
#         else:
#             self.player_pos_ = Pawn(tx, ty)
#         return True

#     def _apply_wall(self, move: Move) -> bool:
#         # Check wall budget
#         if self.current_turn == "bot" and self.bot_walls_remaining <= 0:
#             return False
#         if self.current_turn == "player" and self.player_walls_remaining <= 0:
#             return False
#         legal = self.get_legal_wall_placements()
#         legal_set = {(m.wall.pos, m.wall.orientation) for m in legal}
#         if (move.wall.pos, move.wall.orientation) not in legal_set:
#             return False
#         self.walls.append(move.wall)
#         if self.current_turn == "bot":
#             self.bot_walls_remaining -= 1
#         else:
#             self.player_walls_remaining -= 1
#         return True

#     # ------------------------------------------------------------------
#     # Win detection
#     # ------------------------------------------------------------------

#     def _check_win(self):
#         # Bot wins by reaching the far row (y = n-1)
#         if self.bot_pos_.y == self.n_ - 1:
#             self.game_status = "bot_wins"
#         # Player wins by reaching row 0
#         elif self.player_pos_.y == 0:
#             self.game_status = "player_wins"

#     # ------------------------------------------------------------------
#     # Legal move generation
#     # ------------------------------------------------------------------

#     def get_legal_pawn_moves(self) -> list[Move]:
#         """Return all legal pawn moves for the current player."""
#         if self.current_turn == "bot":
#             cur = self.bot_pos_
#             opp = self.player_pos_
#         else:
#             cur = self.player_pos_
#             opp = self.bot_pos_

#         moves = []
#         for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
#             nx, ny = cur.x + dx, cur.y + dy
#             if not (0 <= nx < self.n_ and 0 <= ny < self.n_):
#                 continue
#             if self._wall_blocks(cur.x, cur.y, nx, ny):
#                 continue
#             # If occupied by opponent, try to jump
#             if nx == opp.x and ny == opp.y:
#                 jx, jy = nx + dx, ny + dy
#                 if (0 <= jx < self.n_ and 0 <= jy < self.n_
#                         and not self._wall_blocks(nx, ny, jx, jy)):
#                     moves.append(Move(MoveType.PAWN, target=Pawn(jx, jy)))
#                 else:
#                     # Try diagonal jumps
#                     for sdx, sdy in [(dx, 0), (0, dy)] if dx != 0 else [(1, 0), (-1, 0)]:
#                         if sdx == dx and sdy == dy:
#                             continue
#                         sx, sy = nx + sdx, ny + sdy
#                         if (0 <= sx < self.n_ and 0 <= sy < self.n_
#                                 and not self._wall_blocks(nx, ny, sx, sy)):
#                             moves.append(Move(MoveType.PAWN, target=Pawn(sx, sy)))
#             else:
#                 moves.append(Move(MoveType.PAWN, target=Pawn(nx, ny)))
#         return moves

#     def get_legal_wall_placements(self) -> list[Move]:
#         """Return all legal wall placements for the current player."""
#         walls = []
#         remaining = (self.bot_walls_remaining if self.current_turn == "bot"
#                      else self.player_walls_remaining)
#         if remaining <= 0:
#             return walls

#         wall_n = self.n_ - 1
#         for wx in range(wall_n):
#             for wy in range(wall_n):
#                 for orient in (Orientation.HOR, Orientation.VER):
#                     w = Wall(pos=(wx, wy), orientation=orient)
#                     if not self._wall_overlaps(w) and not self._wall_blocks_all_paths(w):
#                         walls.append(Move(MoveType.WALL, wall=w))
#         return walls

#     # ------------------------------------------------------------------
#     # Wall collision helpers
#     # ------------------------------------------------------------------

#     def _wall_blocks(self, x1, y1, x2, y2) -> bool:
#         """Does any placed wall block movement from (x1,y1) to (x2,y2)?"""
#         dx, dy = x2 - x1, y2 - y1
#         for w in self.walls:
#             wx, wy = w.pos
#             if dy == 1:   # moving up (increasing y)
#                 if w.orientation == Orientation.HOR:
#                     if (wx == x1 and wy == y1) or (wx == x1 - 1 and wy == y1):
#                         return True
#             elif dy == -1:  # moving down
#                 if w.orientation == Orientation.HOR:
#                     if (wx == x1 and wy == y1 - 1) or (wx == x1 - 1 and wy == y1 - 1):
#                         return True
#             elif dx == 1:   # moving right
#                 if w.orientation == Orientation.VER:
#                     if (wx == x1 and wy == y1) or (wx == x1 and wy == y1 - 1):
#                         return True
#             elif dx == -1:  # moving left
#                 if w.orientation == Orientation.VER:
#                     if (wx == x1 - 1 and wy == y1) or (wx == x1 - 1 and wy == y1 - 1):
#                         return True
#         return False

#     def _wall_overlaps(self, new_wall: Wall) -> bool:
#         """Does new_wall overlap or cross any existing wall?"""
#         wx, wy = new_wall.pos
#         for w in self.walls:
#             ex, ey = w.pos
#             if w.orientation == new_wall.orientation:
#                 # Same orientation: overlaps if anchors are adjacent
#                 if new_wall.orientation == Orientation.HOR:
#                     if ey == wy and abs(ex - wx) <= 1:
#                         return True
#                 else:
#                     if ex == wx and abs(ey - wy) <= 1:
#                         return True
#             else:
#                 # Cross: HOR and VER cross at the same anchor
#                 if ex == wx and ey == wy:
#                     return True
#         return False

#     def _wall_blocks_all_paths(self, new_wall: Wall) -> bool:
#         """
#         Returns True if placing new_wall would trap either pawn with no
#         path to their goal row. Uses BFS.
#         """
#         # Temporarily add the wall
#         self.walls.append(new_wall)
#         bot_ok = self._has_path(self.bot_pos_, target_y=self.n_ - 1)
#         player_ok = self._has_path(self.player_pos_, target_y=0)
#         self.walls.pop()
#         return not (bot_ok and player_ok)

#     def _has_path(self, start: Pawn, target_y: int) -> bool:
#         """BFS: can `start` reach any cell with y == target_y?"""
#         visited = set()
#         queue = [(start.x, start.y)]
#         visited.add((start.x, start.y))
#         while queue:
#             x, y = queue.pop(0)
#             if y == target_y:
#                 return True
#             for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
#                 nx, ny = x + dx, y + dy
#                 if (0 <= nx < self.n_ and 0 <= ny < self.n_
#                         and (nx, ny) not in visited
#                         and not self._wall_blocks(x, y, nx, ny)):
#                     visited.add((nx, ny))
#                     queue.append((nx, ny))
#         return False

# quoridor_utils.py
"""
Standalone replacement for quoridor_game.quoridor_utils.
No ROS required — drop this file next to encoder.py, agent.py, etc.
"""
import json
import heapq
from copy import deepcopy
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional
import math  # kept for any callers that expect it


class Orientation(Enum):
    HOR = 0
    VER = 1


class MoveType(Enum):
    WALL = 0
    PAWN = 1


@dataclass
class Wall:
    pos: tuple[int, int]
    orientation: Orientation

    def to_dict(self):
        return {"pos": list(self.pos), "orientation": self.orientation.name}

    @staticmethod
    def from_dict(d):
        return Wall(pos=tuple(d["pos"]), orientation=Orientation[d["orientation"]])


@dataclass
class Pawn:
    x: int
    y: int

    def to_dict(self):
        return {"x": self.x, "y": self.y}

    @staticmethod
    def from_dict(d):
        return Pawn(x=d["x"], y=d["y"])


@dataclass
class Move:
    move_type: MoveType
    target: Optional[Pawn] = None
    wall: Optional[Wall] = None

    def to_dict(self):
        d = {"move_type": self.move_type.name}
        if self.target is not None:
            d["target"] = self.target.to_dict()
        if self.wall is not None:
            d["wall"] = self.wall.to_dict()
        return d

    @staticmethod
    def from_dict(d):
        return Move(
            move_type=MoveType[d["move_type"]],
            target=Pawn.from_dict(d["target"]) if "target" in d and d["target"] else None,
            wall=Wall.from_dict(d["wall"]) if "wall" in d and d["wall"] else None,
        )


class QuoridorBoard:
    WALLS_PER_PLAYER = 4

    def __init__(self, n=5, walls=4):
        self.n_ = n
        self.wall_n_ = walls
        self.bot_pos_ = Pawn(n // 2, 0)
        self.player_pos_ = Pawn(n // 2, n - 1)
        self.walls: list[Wall] = []
        self.bot_walls_remaining = self.WALLS_PER_PLAYER
        self.player_walls_remaining = self.WALLS_PER_PLAYER
        self.current_turn = "player"
        self.game_status = "in_progress"

    def copy(self):
        return deepcopy(self)

    # ------------------------------------------------------------------ #
    #  Wall-blocking queries                                               #
    # ------------------------------------------------------------------ #

    @staticmethod
    def is_blocked_by_wall(wall: Wall, from_pos: Pawn, to_pos: Pawn) -> bool:
        wx, wy = wall.pos
        if wall.orientation == Orientation.HOR:
            if from_pos.x == to_pos.x and from_pos.y != to_pos.y:
                min_y = min(from_pos.y, to_pos.y)
                if wy == min_y and from_pos.x in (wx, wx + 1):
                    return True
        else:
            if from_pos.y == to_pos.y and from_pos.x != to_pos.x:
                min_x = min(from_pos.x, to_pos.x)
                if wx == min_x and from_pos.y in (wy, wy + 1):
                    return True
        return False

    def is_move_blocked(self, from_pos: Pawn, to_pos: Pawn) -> bool:
        for wall in self.walls:
            if self.is_blocked_by_wall(wall, from_pos, to_pos):
                return True
        return False

    # ------------------------------------------------------------------ #
    #  Pawn-move validation                                               #
    # ------------------------------------------------------------------ #

    def is_pawn_move_legal(self, current: Pawn, target: Pawn, opponent: Pawn) -> bool:
        if not (0 <= target.x < self.n_ and 0 <= target.y < self.n_):
            return False
        if current == target:
            return False

        dx = target.x - current.x
        dy = target.y - current.y
        manhattan = abs(dx) + abs(dy)

        if manhattan == 1:
            if target == opponent:
                return False
            return not self.is_move_blocked(current, target)

        if manhattan == 2 and (dx == 0 or dy == 0):
            mid = Pawn(current.x + dx // 2, current.y + dy // 2)
            if mid != opponent:
                return False
            if self.is_move_blocked(current, mid):
                return False
            if self.is_move_blocked(mid, target):
                return False
            return True

        if abs(dx) == 1 and abs(dy) == 1:
            h_mid = Pawn(current.x + dx, current.y)
            if h_mid == opponent and not self.is_move_blocked(current, h_mid):
                behind = Pawn(current.x + 2 * dx, current.y)
                straight_open = (
                    0 <= behind.x < self.n_
                    and not self.is_move_blocked(h_mid, behind)
                )
                if not straight_open and not self.is_move_blocked(h_mid, target):
                    return True

            v_mid = Pawn(current.x, current.y + dy)
            if v_mid == opponent and not self.is_move_blocked(current, v_mid):
                behind = Pawn(current.x, current.y + 2 * dy)
                straight_open = (
                    0 <= behind.y < self.n_
                    and not self.is_move_blocked(v_mid, behind)
                )
                if not straight_open and not self.is_move_blocked(v_mid, target):
                    return True

            return False

        return False

    # ------------------------------------------------------------------ #
    #  Wall-placement validation                                          #
    # ------------------------------------------------------------------ #

    @staticmethod
    def walls_conflict(w1: Wall, w2: Wall) -> bool:
        if w1.orientation == w2.orientation:
            if w1.orientation == Orientation.HOR:
                return w1.pos[1] == w2.pos[1] and abs(w1.pos[0] - w2.pos[0]) <= 1
            else:
                return w1.pos[0] == w2.pos[0] and abs(w1.pos[1] - w2.pos[1]) <= 1
        else:
            return w1.pos == w2.pos

    def is_wall_placement_legal(self, wall: Wall, placing_player: str) -> bool:
        wx, wy = wall.pos
        if not (0 <= wx < self.wall_n_ and 0 <= wy < self.wall_n_):
            return False
        if placing_player == "bot" and self.bot_walls_remaining <= 0:
            return False
        if placing_player == "player" and self.player_walls_remaining <= 0:
            return False
        for existing in self.walls:
            if self.walls_conflict(wall, existing):
                return False
        test_board = self.copy()
        test_board.walls.append(wall)
        if test_board.shortest_path_length(test_board.bot_pos_, test_board.n_ - 1) is None:
            return False
        if test_board.shortest_path_length(test_board.player_pos_, 0) is None:
            return False
        return True

    # ------------------------------------------------------------------ #
    #  A* shortest path                                                   #
    # ------------------------------------------------------------------ #

    def get_neighbors(self, pos: Pawn) -> list[Pawn]:
        neighbors = []
        for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            nb = Pawn(pos.x + dx, pos.y + dy)
            if 0 <= nb.x < self.n_ and 0 <= nb.y < self.n_:
                if not self.is_move_blocked(pos, nb):
                    neighbors.append(nb)
        return neighbors

    def shortest_path_length(self, start: Pawn, goal_row: int) -> Optional[int]:
        if start.y == goal_row:
            return 0
        counter = 0
        open_set = [(abs(start.y - goal_row), counter, start.x, start.y)]
        g_score: dict[tuple[int, int], int] = {(start.x, start.y): 0}

        while open_set:
            f, _, x, y = heapq.heappop(open_set)
            g = g_score.get((x, y), float("inf"))
            if f > g + abs(y - goal_row):
                continue
            if y == goal_row:
                return g
            for nb in self.get_neighbors(Pawn(x, y)):
                new_g = g + 1
                key = (nb.x, nb.y)
                if new_g < g_score.get(key, float("inf")):
                    g_score[key] = new_g
                    counter += 1
                    heapq.heappush(open_set, (new_g + abs(nb.y - goal_row), counter, nb.x, nb.y))
        return None

    def shortest_path(self, start: Pawn, goal_row: int) -> Optional[list[Pawn]]:
        if start.y == goal_row:
            return [start]
        counter = 0
        open_set = [(abs(start.y - goal_row), counter, start.x, start.y)]
        g_score: dict[tuple[int, int], int] = {(start.x, start.y): 0}
        came_from: dict[tuple[int, int], tuple[int, int]] = {}

        while open_set:
            f, _, x, y = heapq.heappop(open_set)
            g = g_score.get((x, y), float("inf"))
            if f > g + abs(y - goal_row):
                continue
            if y == goal_row:
                path = [Pawn(x, y)]
                cur = (x, y)
                while cur in came_from:
                    cur = came_from[cur]
                    path.append(Pawn(cur[0], cur[1]))
                path.reverse()
                return path
            for nb in self.get_neighbors(Pawn(x, y)):
                new_g = g + 1
                key = (nb.x, nb.y)
                if new_g < g_score.get(key, float("inf")):
                    g_score[key] = new_g
                    came_from[key] = (x, y)
                    counter += 1
                    heapq.heappush(open_set, (new_g + abs(nb.y - goal_row), counter, nb.x, nb.y))
        return None

    # ------------------------------------------------------------------ #
    #  Move generation                                                    #
    # ------------------------------------------------------------------ #

    def get_legal_pawn_moves(self) -> list[Move]:
        if self.current_turn == "bot":
            current, opponent = self.bot_pos_, self.player_pos_
        else:
            current, opponent = self.player_pos_, self.bot_pos_

        moves = []
        for dx in range(-2, 3):
            for dy in range(-2, 3):
                if dx == 0 and dy == 0:
                    continue
                t = Pawn(current.x + dx, current.y + dy)
                if 0 <= t.x < self.n_ and 0 <= t.y < self.n_:
                    if self.is_pawn_move_legal(current, t, opponent):
                        moves.append(Move(move_type=MoveType.PAWN, target=t))
        return moves

    def get_legal_wall_placements(self) -> list[Move]:
        player = self.current_turn
        if (player == "bot" and self.bot_walls_remaining <= 0) or \
           (player == "player" and self.player_walls_remaining <= 0):
            return []
        moves = []
        for x in range(self.wall_n_):
            for y in range(self.wall_n_):
                for orient in (Orientation.HOR, Orientation.VER):
                    wall = Wall(pos=(x, y), orientation=orient)
                    if self.is_wall_placement_legal(wall, player):
                        moves.append(Move(move_type=MoveType.WALL, wall=wall))
        return moves

    def get_strategic_wall_placements(self) -> list[Move]:
        player = self.current_turn
        remaining = (self.bot_walls_remaining if player == "bot"
                     else self.player_walls_remaining)
        if remaining <= 0:
            return []

        if player == "bot":
            opp_path = self.shortest_path(self.player_pos_, 0)
        else:
            opp_path = self.shortest_path(self.bot_pos_, self.n_ - 1)

        if opp_path is None or len(opp_path) < 2:
            return []

        candidate_walls: set[tuple[int, int, int]] = set()
        for i in range(len(opp_path) - 1):
            a, b = opp_path[i], opp_path[i + 1]
            dx = b.x - a.x
            dy = b.y - a.y

            if dy != 0:
                min_y = min(a.y, b.y)
                for wx in (a.x - 1, a.x):
                    if 0 <= wx < self.wall_n_ and 0 <= min_y < self.wall_n_:
                        candidate_walls.add((wx, min_y, Orientation.HOR.value))
            if dx != 0:
                min_x = min(a.x, b.x)
                for wy in (a.y - 1, a.y):
                    if 0 <= min_x < self.wall_n_ and 0 <= wy < self.wall_n_:
                        candidate_walls.add((min_x, wy, Orientation.VER.value))

        moves = []
        for wx, wy, ov in candidate_walls:
            wall = Wall(pos=(wx, wy), orientation=Orientation(ov))
            if self.is_wall_placement_legal(wall, player):
                moves.append(Move(move_type=MoveType.WALL, wall=wall))
        return moves

    # ------------------------------------------------------------------ #
    #  Apply move                                                         #
    # ------------------------------------------------------------------ #

    def apply_move(self, move: Move) -> bool:
        if self.current_turn == "bot":
            current, opponent = self.bot_pos_, self.player_pos_
        else:
            current, opponent = self.player_pos_, self.bot_pos_

        if move.move_type == MoveType.PAWN:
            if not self.is_pawn_move_legal(current, move.target, opponent):
                return False
            if self.current_turn == "bot":
                self.bot_pos_ = move.target
            else:
                self.player_pos_ = move.target

        elif move.move_type == MoveType.WALL:
            if not self.is_wall_placement_legal(move.wall, self.current_turn):
                return False
            self.walls.append(move.wall)
            if self.current_turn == "bot":
                self.bot_walls_remaining -= 1
            else:
                self.player_walls_remaining -= 1

        if self.current_turn == "bot" and self.bot_pos_.y == self.n_ - 1:
            self.game_status = "bot_wins"
        elif self.current_turn == "player" and self.player_pos_.y == 0:
            self.game_status = "player_wins"

        self.current_turn = "player" if self.current_turn == "bot" else "bot"
        return True

    # ------------------------------------------------------------------ #
    #  Serialization                                                      #
    # ------------------------------------------------------------------ #

    def to_dict(self) -> dict:
        return {
            "n": self.n_,
            "bot_pos": self.bot_pos_.to_dict(),
            "player_pos": self.player_pos_.to_dict(),
            "walls": [w.to_dict() for w in self.walls],
            "bot_walls_remaining": self.bot_walls_remaining,
            "player_walls_remaining": self.player_walls_remaining,
            "current_turn": self.current_turn,
            "game_status": self.game_status,
        }

    def to_json(self) -> str:
        return json.dumps(self.to_dict())

    @staticmethod
    def from_dict(d: dict) -> "QuoridorBoard":
        board = QuoridorBoard.__new__(QuoridorBoard)
        board.n_ = d["n"]
        board.wall_n_ = d["n"] - 1
        board.bot_pos_ = Pawn.from_dict(d["bot_pos"])
        board.player_pos_ = Pawn.from_dict(d["player_pos"])
        board.walls = [Wall.from_dict(w) for w in d["walls"]]
        board.bot_walls_remaining = d["bot_walls_remaining"]
        board.player_walls_remaining = d["player_walls_remaining"]
        board.current_turn = d["current_turn"]
        board.game_status = d["game_status"]
        return board

    @staticmethod
    def from_json(json_str: str) -> "QuoridorBoard":
        return QuoridorBoard.from_dict(json.loads(json_str))

    # ------------------------------------------------------------------ #
    #  ASCII display                                                      #
    # ------------------------------------------------------------------ #

    def display(self) -> str:
        lines = []
        col_header = "    " + "   ".join(str(i) for i in range(self.n_))
        lines.append(col_header)
        lines.append("  +" + "---+" * self.n_)

        for y in range(self.n_ - 1, -1, -1):
            row = f"{y} |"
            for x in range(self.n_):
                if self.bot_pos_.x == x and self.bot_pos_.y == y:
                    cell = " B "
                elif self.player_pos_.x == x and self.player_pos_.y == y:
                    cell = " P "
                else:
                    cell = "   "
                if x < self.n_ - 1:
                    has_vwall = self.is_move_blocked(Pawn(x, y), Pawn(x + 1, y))
                    row += cell + ("\u2551" if has_vwall else "|")
                else:
                    row += cell + "|"
            lines.append(row)

            if y > 0:
                sep = "  +"
                for x in range(self.n_):
                    has_hwall = self.is_move_blocked(Pawn(x, y), Pawn(x, y - 1))
                    sep += ("===" if has_hwall else "---")
                    sep += "+"
                lines.append(sep)
            else:
                lines.append("  +" + "---+" * self.n_)

        return "\n".join(lines)