from typing import Tuple
from wall_perception import Walls

Primitive = Tuple[str, int]


def right_hand_policy(walls: Walls) -> Primitive:
    if not walls.front:
        return ("FWD", 1)
    elif not walls.right:
        return ("TURN", +90)
    else:
        return ("TURN", -90)
