import math
from dataclasses import dataclass
from enum import Enum

class Orientation(Enum):
    HOR = 0
    VER = 1

@dataclass
class Wall:
    pos: tuple[int, int]
    orientation: Orientation

@dataclass
class Pawn:
    x: int
    y: int

class QuoridorBoard():
    n_: int             # Size of board (n x n). n must be an even number
    wall_n_: int        # Size of board spaces for walls
    bot_pos_: Pawn      # Position of the bot's pawn
    player_pos_: Pawn   # Position of the player's pawn
    walls: list[Wall]   # Wall positions with pos a, b
    
    def __init__(self, n):
        self.n_ = n
        self.wall_n_ = n-1
        self.bot_pos_ = ((n-1)/2, 0)
        self.player_pos_ = ((n-1)/2, n-1)



    def isPawnMoveLegal(self, current_pawn: Pawn, updated_pawn: Pawn, opponent_pawn: Pawn) -> bool:
        
        def isBlockedByWall(wall: Wall, current_pawn: Pawn, updated_pawn: Pawn):
            # If horizontal move
            if(current_pawn.y != updated_pawn.y):
                if( (wall.x == current_pawn.x) and (wall.y == min(current_pawn.y, updated_pawn.y)) ):
                    return True

            # Else-if vertical move\
            else:
                if( (wall.y == current_pawn.y) and (wall.x == min(current_pawn.x, updated_pawn.x)) ):
                    return True

            return False
        
        # Is updated pawn out of bounds?
        if(
            updated_pawn.x >= self.n_ 
            or updated_pawn.x < 0 
            or updated_pawn.y >= self.n_ 
            or updated_pawn.y < 0
            ): return False

        # Is no move done?
        if(current_pawn == updated_pawn): return False

        move_distance = math.dist((current_pawn.x, current_pawn.y), (updated_pawn.x, updated_pawn.y))

        # Is more than 2 steps?
        if(move_distance > 2): return False

        # Is more than 1 step? (i.e. Jumping over a player)
        if(move_distance > 1):
            # If going behind, is it accessible?
            if(move_distance == 2):
                # Is the opponent actually blocking?
                dir_unit_vector: tuple[int, int] = (updated_pawn.x - current_pawn.x, updated_pawn.y - current_pawn.y) / move_distance
                if(opponent_pawn.x != current_pawn.x+dir_unit_vector[0] or opponent_pawn.y != current_pawn.y+dir_unit_vector[1]):
                    return False
                
                # Is blocked by wall?
                for wall in self.walls:
                    if isBlockedByWall(wall, opponent_pawn, updated_pawn):
                        return False

            # If going diagonally, opponent must be adjacent in one orthogonal component of the
            # diagonal, the straight-line path behind them must be blocked (wall or board edge),
            # and neither leg of the diagonal may be wall-blocked.
            else:
                dx = updated_pawn.x - current_pawn.x  # ±1
                dy = updated_pawn.y - current_pawn.y  # ±1

                horiz_mid = Pawn(current_pawn.x + dx, current_pawn.y)
                vert_mid  = Pawn(current_pawn.x,      current_pawn.y + dy)

                if opponent_pawn == horiz_mid:
                    # Path from current to opponent must be clear
                    for wall in self.walls:
                        if isBlockedByWall(wall, current_pawn, horiz_mid):
                            return False
                    # Straight continuation past opponent must be blocked
                    behind = Pawn(current_pawn.x + 2 * dx, current_pawn.y)
                    straight_open = (
                        0 <= behind.x < self.n_
                        and not any(isBlockedByWall(w, horiz_mid, behind) for w in self.walls)
                    )
                    if straight_open:
                        return False
                    # Path from opponent to diagonal target must be clear
                    for wall in self.walls:
                        if isBlockedByWall(wall, horiz_mid, updated_pawn):
                            return False

                elif opponent_pawn == vert_mid:
                    # Path from current to opponent must be clear
                    for wall in self.walls:
                        if isBlockedByWall(wall, current_pawn, vert_mid):
                            return False
                    # Straight continuation past opponent must be blocked
                    behind = Pawn(current_pawn.x, current_pawn.y + 2 * dy)
                    straight_open = (
                        0 <= behind.y < self.n_
                        and not any(isBlockedByWall(w, vert_mid, behind) for w in self.walls)
                    )
                    if straight_open:
                        return False
                    # Path from opponent to diagonal target must be clear
                    for wall in self.walls:
                        if isBlockedByWall(wall, vert_mid, updated_pawn):
                            return False

                else:
                    # Opponent is not adjacent in the right direction for a diagonal jump
                    return False

        # If only one step, is it accessible?
        else:
            # Check if pawn is there
            if(updated_pawn == opponent_pawn): return False
            # Check if wall is there
            for wall in self.walls:
                if isBlockedByWall(wall, current_pawn, updated_pawn):
                    return False

        return True

