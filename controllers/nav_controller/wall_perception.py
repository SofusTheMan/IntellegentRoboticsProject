from dataclasses import dataclass

@dataclass
class Walls:
    front: bool
    right: bool
    left: bool
    behind: bool


class WallPerception:
    def __init__(self, threshold=80.0):
        self.threshold = threshold

    def classify(self, ir_values):
        FRONT = max(ir_values[0], ir_values[7])
        RIGHT = max(ir_values[2], ir_values[3])
        LEFT = max(ir_values[5], ir_values[6])
        BEHIND = max(ir_values[1], ir_values[4])

        return Walls(
            front=FRONT >= self.threshold,
            right=RIGHT >= self.threshold,
            left=LEFT >= self.threshold,
            behind=BEHIND >= self.threshold,
        )
