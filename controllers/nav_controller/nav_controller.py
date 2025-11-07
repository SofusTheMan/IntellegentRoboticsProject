
from controller import Robot
from motion_primitives import MotionPrimitives
from sensors import IRSuite
from wall_perception import WallPerception
from simple_policy import right_hand_policy

LEFT_MOTOR  = "left wheel motor"
RIGHT_MOTOR = "right wheel motor"

# ---- Tune these ----
CELL_SIZE_M  = 0.12   # meters per grid cell 
IR_THRESHOLD = 80.0   # lower than 80 so “far walls” read as false; tune 45–65
# --------------------

robot = Robot()
dt = int(robot.getBasicTimeStep())

# Motors
left  = robot.getDevice(LEFT_MOTOR)
right = robot.getDevice(RIGHT_MOTOR)

# Building blocks
mp  = MotionPrimitives(robot, left, right)
irs = IRSuite(robot)
wp  = WallPerception(threshold=IR_THRESHOLD)  

# Safety limit so it doesn't loop forever during tuning
steps, MAX_STEPS = 0, 600

while robot.step(dt) != -1 and steps < MAX_STEPS:
    ir = irs.read()
    # Debug once if needed:
    print("IR:", ["{:.1f}".format(v) for v in ir])

    walls = wp.classify(ir)
    print("Walls:", walls)

    cmds = right_hand_policy(walls)

    if cmds[0] == "TURN":
        if cmds[1] > 0:
            mp.turn_90_right()
        else:
            mp.turn_90_left()
    elif cmds[0] == "FWD":
        mp.forward_cells(int(cmds[1]), CELL_SIZE_M)

    steps += 1



