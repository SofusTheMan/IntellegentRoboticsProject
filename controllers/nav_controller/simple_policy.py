
from typing import Tuple

# A primitive is ("TURN", ±90) or ("FWD", cells)
Primitive = Tuple[str, int]

def right_hand_policy(walls) -> Primitive:
    """
    If no wall on the right -> turn right.
    Else if no wall in front -> go forward one cell.
    Else -> turn left (until you find space).
    """
    if not walls.right:
        return ("TURN", +90)
    elif not walls.front:
        return ("FWD", 1)
    else:
        return ("TURN", -90)
    

