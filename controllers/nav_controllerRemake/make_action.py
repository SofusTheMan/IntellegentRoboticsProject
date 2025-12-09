STATE_FORWARD = 0
STATE_TURN_RIGHT = 1
STATE_TURN_LEFT = 2

def make_action(action, possible_positions, mapeSize):
    new_positions = []

    for (r, c, orient) in possible_positions:
        nr, nc, no = r, c, orient

        if action == STATE_FORWARD:
            if no == 0:   # NORTH
                nc += 1
            elif no == 1: # EAST
                nr += 1
            elif no == 2: # SOUTH
                nc -= 1
            elif no == 3: # WEST
                nr -= 1

        elif action == STATE_TURN_RIGHT:
            no = (no + 1) % 4

        elif action == STATE_TURN_LEFT:
            no = (no - 1) % 4

        new_positions.append((nr, nc, no))

    return new_positions
