"""
Optimal maze solver for DFS-generated mazes.
Since DFS creates a spanning tree (no loops), there's exactly one path to the exit.
We use BFS to find the shortest path from current position to exit.
"""

from collections import deque

STATE_FORWARD = 0
STATE_TURN_RIGHT = 1
STATE_TURN_LEFT = 2

NORTH, EAST, SOUTH, WEST = 0, 1, 2, 3

def find_exit_cell(maze_graph):
    """Find the cell that has an exit (open wall to outside)."""
    for (x, y), walls in maze_graph.cells.items():
        # Check if any wall is open to outside
        if not walls["N"] and y == maze_graph.cols - 1:
            return (x, y)
        if not walls["S"] and y == 0:
            return (x, y)
        if not walls["E"] and x == maze_graph.rows - 1:
            return (x, y)
        if not walls["W"] and x == 0:
            return (x, y)
    return None

def get_neighbor_cells(maze_graph, x, y):
    """Get accessible neighbor cells (no wall between)."""
    neighbors = []
    cell_walls = maze_graph.cells[(x, y)]
    
    # Check each direction
    if not cell_walls["N"] and y + 1 < maze_graph.cols:
        neighbors.append((x, y + 1, "N"))
    if not cell_walls["S"] and y - 1 >= 0:
        neighbors.append((x, y - 1, "S"))
    if not cell_walls["E"] and x + 1 < maze_graph.rows:
        neighbors.append((x + 1, y, "E"))
    if not cell_walls["W"] and x - 1 >= 0:
        neighbors.append((x - 1, y, "W"))
    
    return neighbors

def bfs_shortest_path(maze_graph, start_pos):
    """
    Find shortest path from start position to exit using BFS.
    Returns list of (x, y, direction) positions to follow.
    """
    x, y, orient = start_pos
    exit_cell = find_exit_cell(maze_graph)
    
    if exit_cell is None:
        return None
    
    # BFS to find shortest path
    queue = deque([(x, y, [(x, y)])])
    visited = {(x, y)}
    
    while queue:
        curr_x, curr_y, path = queue.popleft()
        
        # Check if we reached exit
        if (curr_x, curr_y) == exit_cell:
            return path
        
        # Explore neighbors
        for nx, ny, direction in get_neighbor_cells(maze_graph, curr_x, curr_y):
            if (nx, ny) not in visited:
                visited.add((nx, ny))
                queue.append((nx, ny, path + [(nx, ny)]))
    
    return None

def direction_to_move(from_cell, to_cell):
    """Determine which direction to move to go from one cell to another."""
    x1, y1 = from_cell
    x2, y2 = to_cell
    
    if x2 > x1:  # Moving right
        return "E"
    elif x2 < x1:  # Moving left
        return "W"
    elif y2 > y1:  # Moving up
        return "N"
    elif y2 < y1:  # Moving down
        return "S"
    return None

def orientation_to_direction_map():
    """Map orientation integers to direction strings."""
    return {
        NORTH: "N",
        EAST: "E",
        SOUTH: "S",
        WEST: "W"
    }

def direction_to_orientation_map():
    """Map direction strings to orientation integers."""
    return {
        "N": NORTH,
        "E": EAST,
        "S": SOUTH,
        "W": WEST
    }

def turns_needed(current_orient, target_orient):
    """
    Calculate optimal turn action to face target direction.
    Returns (action, num_turns) where action is STATE_TURN_LEFT or STATE_TURN_RIGHT.
    """
    # Calculate shortest rotation
    diff = (target_orient - current_orient) % 4
    
    if diff == 0:
        return None, 0  # Already facing correct direction
    elif diff == 1:
        return STATE_TURN_RIGHT, 1  # Turn right once
    elif diff == 2:
        return STATE_TURN_RIGHT, 2  # Turn right twice (or left twice, same cost)
    elif diff == 3:
        return STATE_TURN_LEFT, 1  # Turn left once (same as right 3 times)
    
def next_action_optimal(maze_graph, position):
    """
    Decide the optimal next action to solve the maze fastest.
    Uses BFS to find shortest path, then determines next move.
    """
    x, y, orient = position
    
    # Get shortest path to exit
    path = bfs_shortest_path(maze_graph, position)
    
    if path is None or len(path) < 2:
        # No path found or already at exit
        return None
    
    # Next cell to move to
    next_cell = path[1]  # path[0] is current cell
    
    # Determine which direction we need to move
    direction_needed = direction_to_move((x, y), next_cell)
    target_orient = direction_to_orientation_map()[direction_needed]
    
    # Check if we're already facing the right direction
    if orient == target_orient:
        return STATE_FORWARD
    
    # Calculate optimal turn
    action, num_turns = turns_needed(orient, target_orient)
    return action

def next_action_simple(maze_graph, position):
    """
    Simple greedy approach: always try forward first, then turn right.
    This will eventually solve any DFS maze but not optimally.
    """
    x, y, orient = position
    
    # Check if we can move forward
    cell_walls = maze_graph.cells[(x, y)]
    orient_map = orientation_to_direction_map()
    
    current_direction = orient_map[orient]
    
    # If no wall ahead, go forward
    if not cell_walls[current_direction]:
        return STATE_FORWARD
    else:
        # Wall ahead, turn right
        return STATE_TURN_RIGHT

# Example usage:
def get_next_action(maze_graph, position, use_optimal=True):
    """
    Main function to get next action.
    
    Args:
        maze_graph: MazeGraph object
        position: (x, y, orientation) tuple
        use_optimal: If True, uses BFS optimal solver. If False, uses simple greedy approach.
    
    Returns:
        STATE_FORWARD, STATE_TURN_RIGHT, STATE_TURN_LEFT, or None (if at exit)
    """
    if use_optimal:
        return next_action_optimal(maze_graph, position)
    else:
        return next_action_simple(maze_graph, position)