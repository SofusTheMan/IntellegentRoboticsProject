NORTH, EAST, SOUTH, WEST = 0, 1, 2, 3


def update_possible_positions(walls, possible_positions, maze_map):
   # Keep only positions/orientations that match the detected walls.
   
    # Map robot orientation + relative direction → absolute maze direction
    dir_map = {
        NORTH : {"front": "N", "back": "S", "left": "W", "right": "E"},
        SOUTH : {"front": "S", "back": "N", "left": "E", "right": "W"},
        EAST :  {"front": "E", "back": "W", "left": "N", "right": "S"},
        WEST :  {"front": "W", "back": "E", "left": "S", "right": "N"},
    }

    updated_positions = []

    for (r, c, orient) in possible_positions:
        if not maze_map.in_bounds(r, c):
            continue
        cell_walls = maze_map.cells[(r, c)]  # e.g. {"N": True, "S": False, ...}

        absolute_dir = dir_map[orient]

        match = True
        for rel_dir, detected in walls.items():
            maze_wall = cell_walls[absolute_dir[rel_dir]]

            # If robot detects a wall but maze has no wall → impossible
            # If robot detects no wall but maze has a wall → impossible
            if detected != maze_wall:
                match = False
                break

        if match:
            updated_positions.append((r, c, orient))

    return updated_positions
