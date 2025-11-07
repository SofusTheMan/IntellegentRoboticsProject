
from controller import Robot
from motion_primitives import MotionPrimitives
from sensors import IRSuite
from wall_perception import WallPerception
from simple_policy import right_hand_policy


LEFT_MOTOR  = "left wheel motor"
RIGHT_MOTOR = "right wheel motor"

# ===== TUNE THESE TWO =====
CELL_SIZE_M = 0.15   # meters per grid cell (calibrate once)
IR_THRESHOLD = 80.0  # higher => more “sensitive” to walls
# =========================

robot = Robot()
dt = int(robot.getBasicTimeStep())

# Motors
left = robot.getDevice(LEFT_MOTOR)
right = robot.getDevice(RIGHT_MOTOR)

# Building blocks
mp = MotionPrimitives(robot, left, right)
irs = IRSuite(robot)
wp = WallPerception(threshold=IR_THRESHOLD)

# Safety limit so it does not loop forever during tuning
steps = 0
MAX_STEPS = 600

while robot.step(dt) != -1 and steps < MAX_STEPS:
    ir = irs.read()

    # print once to check which indices spike near walls
    # print(ir); break

    walls = wp.classify(ir)
    cmd = right_hand_policy(walls)

    if cmd[0] == "TURN":
        if cmd[1] > 0:   # +90 -> right (CW)
            mp.turn_90_right()
        else:            # -90 -> left (CCW)
            mp.turn_90_left()
    elif cmd[0] == "FWD":
        mp.forward_cells(cmd[1], CELL_SIZE_M)

    steps += 1

