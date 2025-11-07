
from controller import Robot
from motion_primitives import MotionPrimitives
from sensors import IRSuite
from wall_perception import WallPerception
from simple_policy import right_hand_policy

LEFT_MOTOR  = "left wheel motor"
RIGHT_MOTOR = "right wheel motor"

# ---- Tune these ----
CELL_SIZE_M  = 0.12   # meters per grid cell 
IR_THRESHOLD = 55.0   # lower than 80 so “far walls” read as false; tune 45–65
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
    # print("IR:", ["{:.1f}".format(v) for v in ir])

    walls = wp.classify(ir)

    # Work out "all clear" even if the Walls dont have that field yet
    try:
        all_clear = getattr(walls, "all_clear")
    except Exception:
        all_clear = (not walls.front and not walls.right and not walls.left and not walls.behind)

    # Avoid spin when everything is far: move forward one cell
    if all_clear:
        cmds = [("FWD", 1)]
    else:
        cmds = right_hand_policy(walls)
        # right_hand_policy might return a single tuple or a list of tuples
        if isinstance(cmds, tuple):
            cmds = [cmds]

    # Execute the whole sequence (e.g., TURN right THEN FWD 1)
    for kind, val in cmds:
        if kind == "TURN":
            if val > 0:
                mp.turn_90_right()
            else:
                mp.turn_90_left()
        elif kind == "FWD":
            mp.forward_cells(int(val), CELL_SIZE_M)

    steps += 1



