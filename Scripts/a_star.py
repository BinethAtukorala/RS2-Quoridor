from typing import List, Tuple, Dict, Set
import numpy as np
from math import sqrt
import heapq    # Priority queue

# If the board is n x n, the grid: array will be 2n x 2n. The odd numbered indexes will be spaces for walls
# Free spaces as 0
# Player 1 as 1
# Player 2 as 2
# Walls as 3
n = 5
grid_size = 2*n-1
grid = np.zeros((grid_size, grid_size), dtype=int)

def create_node(
        position: Tuple[int, int], # (x, y) coordinates of the note
        g: float = float('inf'),  # Cost from the start
        h: float = 0.0, # Estimated (heuristic) cost to goal
        parent: Dict = None # Parent node
        ) -> Dict:
    return {
        'position': position,
        'g': g,
        'h': h,
        'f': g+h,
        'parent': parent
    }

# Use manhattan distance to estimate distance to goal
def calculate_heuristic(pos1: Tuple[int, int], pos2: Tuple[int, int]) -> float:
    x1, y1 = pos1
    x2, y2 = pos2
    return abs(x1-x2) + abs(y1-y2)

def calculate_heuristic_y(pos1: Tuple[int, int], posy: int) -> float:
    _, y1 = pos1
    return abs(y1 - posy)

# Get all valid neighboring cells in the grid
def get_valid_neighbors(grid: np.ndarray, position: Tuple[int, int]) -> List[Tuple[int, int]]:
    x, y = position
    possible_moves = [
        (x+2, y), (x-2, y),     # Right, Left
        (x, y-2), (x, y+2)      # Up, Down
    ]

    possible_walls = [
        (x+1, y), (x-1, y),     # Right, Left
        (x, y-1), (x, y+1)      # Up, Down
    ]

    return [
        pos for idx, pos in enumerate(possible_moves) # For every move
        if 0 <= pos[0] < grid_size and 0 <= pos[1] < grid_size  # If within bounds
        and grid[possible_walls[idx][0], possible_walls[idx][1]] == 0 # Not a wall
    ]

# Reconstruct path following parents
def reconstruct_path(goal_node: Dict) -> List[Tuple[int, int]]:
    path = []
    current = goal_node

    while current is not None:
        path.append(current['position'])
        current = current['parent']

    return path [::-1] # Reverse to get path from start to goal

# Find optimal path using A*
def find_path(grid: np.ndarray, start: Tuple[int, int],
              goal: Tuple[int, int]) -> List[Tuple[int, int]]:
    # Initialise start node
    start_node = create_node(
        position=start,
        g=0,
        h=calculate_heuristic_y(start, goal[1])
    )

    # Initialise open and closed sets
    open_list = [(start_node['f'], start)]  # Priority queue (estimated full cost, pos)
    open_dict = {start: start_node}         # Node lookup
    closed_set = set()                      # Explored nodes

    while open_list:
        # Get node with lowest f value
        _, current_pos = heapq.heappop(open_list)
        current_node = open_dict[current_pos]

        # Check if goal reached
        if current_pos[1] == goal[1]:
            return reconstruct_path(current_node)
        
        closed_set.add(current_pos)

        # Explore neighbors
        for neighbor_pos in get_valid_neighbors(grid, current_pos):
            # Skip if already explored
            if neighbor_pos in closed_set:
                continue
            
            # Calculate new path cost
            tentative_g =  current_node['g'] + calculate_heuristic(current_pos, neighbor_pos)

            # Create or update neighbor
            if neighbor_pos not in open_dict:
                neighbor = create_node(
                    position=neighbor_pos,
                    g=tentative_g,
                    h=calculate_heuristic_y(neighbor_pos, goal[1]),
                    parent=current_node
                )
                heapq.heappush(open_list, (neighbor['f'], neighbor_pos))
                open_dict[neighbor_pos] = neighbor
            elif tentative_g < open_dict[neighbor_pos]['g']:
                # Found a better path
                neighbor = open_dict[neighbor_pos]
                neighbor['g'] = tentative_g
                neighbor['f'] = tentative_g + neighbor['h']
                neighbor['parent'] = current_node

    return [] # No path found

import matplotlib.pyplot as plt

# Visualise path
def visualize_path(grid: np.ndarray, path: List[Tuple[int, int]]):
    plt.figure(figsize=(10, 10))
    
    # Flip x and y
    grid_flipped = grid.T
    plt.imshow(grid_flipped, cmap='binary')

    # Even index positions
    tick_positions = np.arange(0, 2*n, 2)

    # Compressed labels 
    tick_labels = np.arange(n)

    plt.xticks(tick_positions, tick_labels)
    plt.yticks(tick_positions, tick_labels)
    
    if path:
        path = np.array(path)
        plt.plot(path[:, 0], path[:, 1], 'b-', linewidth=3, label='Path')
        plt.plot(path[0, 0], path[0, 1], 'go', markersize=15, label='Start')
        plt.plot(path[-1, 0], path[-1, 1], 'ro', markersize=15, label='Goal')
    
    plt.grid(True)
    plt.legend(fontsize=12)
    plt.title("A* Pathfinding Result")
    plt.show()


if __name__ == "__main__":
    # Add some walls
    grid[0, 1] = 3
    grid[1, 1] = 3  # vis only
    grid[2, 1] = 3

    grid[3, 2] = 3
    grid[3, 3] = 3  # vis only
    grid[3, 4] = 3

    grid[4, 3] = 3
    grid[5, 3] = 3  # vis only
    grid[6, 3] = 3

    grid[6, 5] = 3
    grid[7, 5] = 3  # vis only
    grid[8, 5] = 3

    start_pos = 0, 0
    goal_pos = 0, 8

    path = find_path(grid, start_pos, goal_pos)
    if path:
        print(f"Path found with {len(path)} steps!")
        visualize_path(grid, path)

    else:
        print("No path found!")