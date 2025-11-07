
from dataclasses import dataclass

@dataclass
class Walls:
    front: bool
    right: bool
    left: bool
    behind: bool

class WallPerception:
    """
    Classifies walls from IR values using a simple threshold.
    You may need to adjust which sensors correspond to F/R/L/B for your robot.
    """
    def __init__(self, threshold=80.0):
        self.threshold = threshold

    def classify(self, ir_values):
        # --- Map IR indices to directions (adjust if needed) ---
        # Common e-puck layout (approx):
        # front: ps0, ps7; right: ps2, ps3; left: ps5, ps6; behind: ps1, ps4
        FRONT  = max(ir_values[0], ir_values[7])
        RIGHT  = max(ir_values[2], ir_values[3])
        LEFT   = max(ir_values[5], ir_values[6])
        BEHIND = max(ir_values[1], ir_values[4])

        return Walls(
            front  = FRONT  >= self.threshold,
            right  = RIGHT  >= self.threshold,
            left   = LEFT   >= self.threshold,
            behind = BEHIND >= self.threshold
        )
