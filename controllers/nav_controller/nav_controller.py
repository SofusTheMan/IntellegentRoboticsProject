from controller import Robot
from motion_primitives import MotionPrimitives
from sensors import IRSuite
from wall_perception import WallPerception
from Localisation import ParticleFilter
from maze_map import MazeMap, NORTH, EAST, SOUTH, WEST
from planner import Planner
import math

LEFT_MOTOR = "left wheel motor"
RIGHT_MOTOR = "right wheel motor"

# Maze parameters (should match maze_generator.py)
MAZE_ROWS = 5
MAZE_COLS = 5
MAZE_SIDE_LENGTH = 0.6  # meters
WALL_CELL_RATIO = 0.1
CELL_SIZE_M = 0.09  # Actual cell size after walls
IR_THRESHOLD = 80.0  # Wall detection threshold

# Goal: Exit detection (edge cell with no wall = exit)
# detects exit dynamically by checking edge cells with open walls
GOAL_ROW = None  # Will be detected
GOAL_COL = None  # Will be detected

# Calculate actual cell dimensions
wall_width = WALL_CELL_RATIO * MAZE_SIDE_LENGTH / MAZE_ROWS
actual_cell_size = (MAZE_SIDE_LENGTH - (MAZE_ROWS + 1) * wall_width) / MAZE_ROWS

robot = Robot()
dt = int(robot.getBasicTimeStep())

# Motors
left_motor = robot.getDevice(LEFT_MOTOR)
right_motor = robot.getDevice(RIGHT_MOTOR)
mp = MotionPrimitives(robot, left_motor, right_motor)

# Sensors
irs = IRSuite(robot, noise_sigma=0.0)
wp = WallPerception(threshold=IR_THRESHOLD)

# Map and Planner
maze_map = MazeMap(MAZE_ROWS, MAZE_COLS)
planner = Planner(rows=MAZE_ROWS, cols=MAZE_COLS)
# Goal will be set when exit is detected

# Particle Filter (with unknown starting position)
pf = ParticleFilter(
    num_particles=200,
    maze_map=maze_map,
    cell_size=CELL_SIZE_M,
    maze_rows=MAZE_ROWS,
    maze_cols=MAZE_COLS,
    maze_side_length=MAZE_SIDE_LENGTH
)

steps = 0
MAX_STEPS = 10000
current_path = []
last_cell = None
exploration_mode = True  # Start in exploration mode until localized
goal_detected = False
moved_at_least_once = False  # Prevent goal check before movement

# Exploration state
exploration_stack = []  # Stack of cells for backtracking
last_visited_cell = None
current_path_cells = []  # Track current path being explored
path_explored = False  # Flag to track if current path is fully explored
current_cell_directions_tried = set()  # Track which directions we've tried at current cell
last_cell_before_turn = None  # Track cell before turning to avoid immediate backtrack

# Loop prevention
last_action = None
action_repeat_count = 0
MAX_ACTION_REPEATS = 3  # If same action repeated 3 times, switch to exploration


def world_to_cell(x, y):
    """Convert world coordinates to cell (row, col)."""
    cell_c = int((x - wall_width) / (actual_cell_size + wall_width))
    cell_r = int((y - wall_width) / (actual_cell_size + wall_width))
    # Clamp to valid range
    cell_r = max(0, min(MAZE_ROWS - 1, cell_r))
    cell_c = max(0, min(MAZE_COLS - 1, cell_c))
    return cell_r, cell_c


def cell_to_world(r, c):
    """Convert cell (row, col) to world coordinates (center of cell)."""
    x = wall_width + c * (actual_cell_size + wall_width) + actual_cell_size / 2.0
    y = wall_width + r * (actual_cell_size + wall_width) + actual_cell_size / 2.0
    return x, y


def update_map_from_walls(cell_r, cell_c, walls, orientation):
    """
    Update map based on detected walls.
    orientation: 0=E, π/2=N, π=W, -π/2=S
    """
    if not maze_map.in_bounds(cell_r, cell_c):
        return
    
    # Normalize orientation
    theta_norm = orientation % (2 * math.pi)
    
    # Map detected walls to map directions based on robot orientation
    if abs(theta_norm) < 0.1 or abs(theta_norm - 2*math.pi) < 0.1:  # East
        if walls.front:
            maze_map.set_wall(cell_r, cell_c, EAST, True)
        else:
            maze_map.set_wall(cell_r, cell_c, EAST, False)
        if walls.right:
            maze_map.set_wall(cell_r, cell_c, SOUTH, True)
        else:
            maze_map.set_wall(cell_r, cell_c, SOUTH, False)
        if walls.left:
            maze_map.set_wall(cell_r, cell_c, NORTH, True)
        else:
            maze_map.set_wall(cell_r, cell_c, NORTH, False)
        if walls.behind:
            maze_map.set_wall(cell_r, cell_c, WEST, True)
        else:
            maze_map.set_wall(cell_r, cell_c, WEST, False)
    elif abs(theta_norm - math.pi/2) < 0.1:  # North
        if walls.front:
            maze_map.set_wall(cell_r, cell_c, NORTH, True)
        else:
            maze_map.set_wall(cell_r, cell_c, NORTH, False)
        if walls.right:
            maze_map.set_wall(cell_r, cell_c, EAST, True)
        else:
            maze_map.set_wall(cell_r, cell_c, EAST, False)
        if walls.left:
            maze_map.set_wall(cell_r, cell_c, WEST, True)
        else:
            maze_map.set_wall(cell_r, cell_c, WEST, False)
        if walls.behind:
            maze_map.set_wall(cell_r, cell_c, SOUTH, True)
        else:
            maze_map.set_wall(cell_r, cell_c, SOUTH, False)
    elif abs(theta_norm - math.pi) < 0.1:  # West
        if walls.front:
            maze_map.set_wall(cell_r, cell_c, WEST, True)
        else:
            maze_map.set_wall(cell_r, cell_c, WEST, False)
        if walls.right:
            maze_map.set_wall(cell_r, cell_c, NORTH, True)
        else:
            maze_map.set_wall(cell_r, cell_c, NORTH, False)
        if walls.left:
            maze_map.set_wall(cell_r, cell_c, SOUTH, True)
        else:
            maze_map.set_wall(cell_r, cell_c, SOUTH, False)
        if walls.behind:
            maze_map.set_wall(cell_r, cell_c, EAST, True)
        else:
            maze_map.set_wall(cell_r, cell_c, EAST, False)
    else:  # South
        if walls.front:
            maze_map.set_wall(cell_r, cell_c, SOUTH, True)
        else:
            maze_map.set_wall(cell_r, cell_c, SOUTH, False)
        if walls.right:
            maze_map.set_wall(cell_r, cell_c, WEST, True)
        else:
            maze_map.set_wall(cell_r, cell_c, WEST, False)
        if walls.left:
            maze_map.set_wall(cell_r, cell_c, EAST, True)
        else:
            maze_map.set_wall(cell_r, cell_c, EAST, False)
        if walls.behind:
            maze_map.set_wall(cell_r, cell_c, NORTH, True)
        else:
            maze_map.set_wall(cell_r, cell_c, NORTH, False)


def get_action_to_cell(current_cell, next_cell, current_theta):
    """
    Determine action needed to move from current_cell to next_cell.
    Returns: ('forward', 0) or ('turn', angle_deg)
    """
    dr = next_cell[0] - current_cell[0]
    dc = next_cell[1] - current_cell[1]
    
    # Determine required orientation
    if dr == -1 and dc == 0:  # North
        required_theta = math.pi / 2
    elif dr == 1 and dc == 0:  # South
        required_theta = -math.pi / 2
    elif dr == 0 and dc == 1:  # East
        required_theta = 0.0
    elif dr == 0 and dc == -1:  # West
        required_theta = math.pi
    else:
        return None
    
    # Normalize current theta to [0, 2π) for easier comparison
    current_theta_norm = current_theta % (2 * math.pi)
    # Normalize required theta to [0, 2π)
    if required_theta < 0:
        required_theta_norm = required_theta + 2 * math.pi
    else:
        required_theta_norm = required_theta
    
    # Calculate angle difference (shortest path)
    angle_diff = required_theta_norm - current_theta_norm
    if angle_diff > math.pi:
        angle_diff -= 2 * math.pi
    elif angle_diff < -math.pi:
        angle_diff += 2 * math.pi
    
    angle_diff_deg = math.degrees(angle_diff)
    

    # This prevents unnecessary turns when robot is already facing roughly the right way
    if abs(angle_diff_deg) < 45:  # threshold (45 degrees)
        return ('forward', 0)
    else:
        # Normalize large turns to 90-degree increments to prevent oscillation
        if abs(angle_diff_deg) > 90:
            # Break into 90-degree steps
            if angle_diff_deg > 0:
                return ('turn', 90)
            else:
                return ('turn', -90)
        else:
            return ('turn', angle_diff_deg)


def check_wall_during_movement():
    """Check if wall is detected during movement - used as callback during forward motion."""
    ir_current = irs.read()
    walls_current = wp.classify(ir_current)
    
    # Stop only if a wall is detected directly in front.
    # relys on the classification threshold (IR_THRESHOLD) and avoid extra raw checks
    # so the robot stays flexible in narrow corridors.
    return walls_current.front


def execute_action(action, walls, ir_readings=None):
    
    action_type, value = action
    
    if action_type == 'forward':
        # Always rechecks sensors before moving forward
        if ir_readings is not None:
            # Reread sensors to ensure no wall (use same threshold as wall detection)
            current_walls = wp.classify(ir_readings)
            if current_walls.front:
                print("  [SAFETY] Front blocked (sensor check), cannot move forward")
                return False
        
        # Double-check with provided walls
        if walls.front:
            print("  [SAFETY] Front blocked, cannot move forward")
            return False
        
        # Move forward with continuous sensor monitoring
        success = mp.forward_cells(1, CELL_SIZE_M, 
                                   check_sensors=check_wall_during_movement,
                                   stop_on_wall=True)
        
        if success:
            pf.predict(forward_dist=CELL_SIZE_M, turn_angle=0.0,
                       sigma_fwd=0.01, sigma_turn=0.02)
            return True
        else:
            print("  [SAFETY] Movement stopped due to wall detection")
            return False
    elif action_type == 'turn':
        turn_angle_rad = math.radians(value)
        # Use optimized 90-degree turns when possible
        if abs(value - 90) < 10:
            mp.turn_90_left()
        elif abs(value + 90) < 10:
            mp.turn_90_right()
        elif abs(abs(value) - 180) < 10:
            # 180 degree turn
            mp.turn_90_right()
            mp.turn_90_right()
        else:
            # Arbitrary angle
            mp.turn_deg(value)
        
        pf.predict(forward_dist=0.0, turn_angle=turn_angle_rad,
                   sigma_fwd=0.0, sigma_turn=0.02)
        return True
    
    return False


def is_localized():
    """Check if particle filter has converged (low uncertainty)."""
    # Calculate spread of particles
    particles_x = [p[0] for p in pf.particles]
    particles_y = [p[1] for p in pf.particles]
    
    if len(particles_x) == 0:
        return False
    
    x_std = math.sqrt(sum((x - sum(particles_x)/len(particles_x))**2 for x in particles_x) / len(particles_x))
    y_std = math.sqrt(sum((y - sum(particles_y)/len(particles_y))**2 for y in particles_y) / len(particles_y))
    
    # Consider localized if std dev is less than one cell
    return x_std < actual_cell_size * 0.5 and y_std < actual_cell_size * 0.5


def detect_exit(cell_r, cell_c, walls, orientation, ir_readings=None):
    """
    Detect exit: edge cell with NO WALL = exit.
    Simple logic: if at edge AND no wall in front = exit.
    Returns (exit_row, exit_col) if exit found, None otherwise.
    """
    # Check if we're at an edge cell
    is_north_edge = (cell_r == 0)
    is_south_edge = (cell_r == MAZE_ROWS - 1)
    is_east_edge = (cell_c == MAZE_COLS - 1)
    is_west_edge = (cell_c == 0)
    
    if not (is_north_edge or is_south_edge or is_east_edge or is_west_edge):
        return None
    
    # First, check the internal map (if we have information) to see if this edge is open.
    if maze_map.in_bounds(cell_r, cell_c):
        cell = maze_map.get_cell(cell_r, cell_c)
        if is_north_edge and not cell.wallN:
            return (cell_r, cell_c)
        if is_south_edge and not cell.wallS:
            return (cell_r, cell_c)
        if is_east_edge and not cell.wallE:
            return (cell_r, cell_c)
        if is_west_edge and not cell.wallW:
            return (cell_r, cell_c)
    
    # As a fallback, rely on sensors: if at edge and no wall detected ahead, treat as exit.
    if not walls.front:
        if ir_readings is not None:
            front_sensors = [ir_readings[0], ir_readings[7]]
            max_front = max(front_sensors)
            if max_front < IR_THRESHOLD * 0.7:  # more tolerant threshold for open space
                return (cell_r, cell_c)
        else:
            return (cell_r, cell_c)
    
    return None


# Main control loop
print("=" * 60)
print("Simultaneous Maze Navigation and Localization Robot")
print("=" * 60)
print(f"Maze: {MAZE_ROWS}x{MAZE_COLS}, Goal: Exit (to be detected)")
print(f"Particles: {pf.num_particles}, Unknown starting position")
print("=" * 60)

while robot.step(dt) != -1 and steps < MAX_STEPS:
    ir = irs.read()
    walls = wp.classify(ir)
    
    # Get estimated pose from particle filter
    est_x, est_y, est_theta = pf.estimate_pose()
    current_cell = world_to_cell(est_x, est_y)
    
    # Update map based on current observations
    update_map_from_walls(current_cell[0], current_cell[1], walls, est_theta)
    maze_map.mark_visited(current_cell[0], current_cell[1])
    
    # Update particle filter with sensor data
    pf.update(ir, walls)
    pf.resample()
    
    # Re-estimate pose after update
    est_x, est_y, est_theta = pf.estimate_pose()
    current_cell = world_to_cell(est_x, est_y)
    
    # Track if we moved this step
    action_executed_this_step = False
    
    
    exit_cell = detect_exit(current_cell[0], current_cell[1], walls, est_theta, ir)
    
    # Detect exit (goal) if not already detected
    if not goal_detected and moved_at_least_once:
        if exit_cell and exit_cell == current_cell:
            GOAL_ROW, GOAL_COL = exit_cell
            planner.set_goal_cells([(GOAL_ROW, GOAL_COL)])
            goal_detected = True
            print(f"\n[EXIT DETECTED] Goal set to cell ({GOAL_ROW}, {GOAL_COL})")
            print(f"  Robot at edge cell, sensors confirm no wall = EXIT!")
    
    # Check if it has reached the exit - (every step, BEFORE and AFTER movement)
    if moved_at_least_once:
        # Check if at exit cell with no wall 
        is_at_exit = False
        front_sensors = [ir[0], ir[7]]
        max_front = max(front_sensors)
        
        # Check if it is at an edge cell
        is_north_edge = (current_cell[0] == 0)
        is_south_edge = (current_cell[0] == MAZE_ROWS - 1)
        is_east_edge = (current_cell[1] == MAZE_COLS - 1)
        is_west_edge = (current_cell[1] == 0)
        is_at_edge = is_north_edge or is_south_edge or is_east_edge or is_west_edge
        
        
        if is_at_edge and not walls.front and max_front < IR_THRESHOLD * 0.4:
            is_at_exit = True
            # Set goal if not already set
            if not goal_detected:
                GOAL_ROW, GOAL_COL = current_cell
                goal_detected = True
                planner.set_goal_cells([(GOAL_ROW, GOAL_COL)])
                print(f"\n[EXIT DETECTED] Goal set to cell ({GOAL_ROW}, {GOAL_COL})")
        
        
        if goal_detected:
            
            if current_cell == (GOAL_ROW, GOAL_COL):
                
                if not walls.front:
                    is_at_exit = True
                
                elif max_front < IR_THRESHOLD * 0.5:
                    is_at_exit = True
            
            
            dr = GOAL_ROW - current_cell[0]  
            dc = GOAL_COL - current_cell[1]  
            if abs(dr) + abs(dc) == 1:  
                
                theta_norm = est_theta % (2 * math.pi)
                facing_goal = False
                
                # Check if orientation matches direction TO goal
                if dr == 1 and abs(theta_norm - math.pi/2) < 0.5:  
                    facing_goal = True
                elif dr == -1 and (abs(theta_norm + math.pi/2) < 0.5 or abs(theta_norm - 3*math.pi/2) < 0.5):  
                    facing_goal = True
                elif dc == 1 and (abs(theta_norm) < 0.5 or abs(theta_norm - 2*math.pi) < 0.5):  
                    facing_goal = True
                elif dc == -1 and abs(theta_norm - math.pi) < 0.5:  
                    facing_goal = True
                
                if facing_goal and not walls.front and max_front < IR_THRESHOLD * 0.4:
                    is_at_exit = True
        
        # checks if at edge with no wall (even if goal not detected yet)
        if exit_cell and exit_cell == current_cell:
            if not walls.front and max_front < IR_THRESHOLD * 0.4:
                is_at_exit = True
                # Set goal if not already set
                if not goal_detected:
                    GOAL_ROW, GOAL_COL = exit_cell
                    goal_detected = True
                    planner.set_goal_cells([(GOAL_ROW, GOAL_COL)])
        
        # If at goal cell and goal is detected, prioritize exit check
        if goal_detected and current_cell == (GOAL_ROW, GOAL_COL):
            # If sensors show no wall, we're done!
            if not walls.front or max_front < IR_THRESHOLD * 0.6:
                is_at_exit = True
        
        if is_at_exit:
            # STOP IMMEDIATELY - Exit reached!
            mp.stop()
            print(f"\n{'='*60}")
            print(f"✓ GOAL REACHED at step {steps}!")
            print(f"Final pose: ({est_x:.3f}, {est_y:.3f}, {math.degrees(est_theta):.1f}°)")
            print(f"Final cell: {current_cell}")
            if goal_detected:
                print(f"Goal cell: ({GOAL_ROW}, {GOAL_COL})")
            print(f"Front sensors: {front_sensors[0]:.1f}, {front_sensors[1]:.1f} (no wall = exit!)")
            print(f"At edge: {is_at_edge}, No wall: {not walls.front}")
            print(f"{'='*60}")
            break
    
    # Decision making, so it chooses action with highest probability of success
    # Use Dijkstra path planning if we have a good estimate, otherwise explore
    
    localized = is_localized()
    
    # If already at goal, don't try to plan - just check if we should declare success
    if goal_detected and current_cell == (GOAL_ROW, GOAL_COL):
        # it is at the goal cell, so sensors check one more time
        front_sensors_check = [ir[0], ir[7]]
        max_front_check = max(front_sensors_check)
        walls_check = wp.classify(ir)
        
        if not walls_check.front or max_front_check < IR_THRESHOLD * 0.6:
            # it s at goal and sensors confirm no wall = EXIT REACHED!
            mp.stop()
            print(f"\n{'='*60}")
            print(f"✓ GOAL REACHED at step {steps}!")
            print(f"Final pose: ({est_x:.3f}, {est_y:.3f}, {math.degrees(est_theta):.1f}°)")
            print(f"Final cell: {current_cell}")
            print(f"Goal cell: ({GOAL_ROW}, {GOAL_COL})")
            print(f"Front sensors: {front_sensors_check[0]:.1f}, {front_sensors_check[1]:.1f} (no wall = exit!)")
            print(f"{'='*60}")
            break
    
    # Only used path planning if goal is detected AND we're not already at goal
    if goal_detected and (localized or steps > 50) and current_cell != (GOAL_ROW, GOAL_COL):  # Use path planning after some exploration
        exploration_mode = False
        
        # Plan path using Dijkstra
        planner.flood_fill(maze_map)  
        path = planner.dijkstra(maze_map, current_cell, (GOAL_ROW, GOAL_COL))
        
        if path and len(path) > 1:
            current_path = path
            next_cell = path[1]  # Nextcell in pathh
            
            
            if next_cell == (GOAL_ROW, GOAL_COL) or current_cell == (GOAL_ROW, GOAL_COL):
                # it's at or moving to exit - checks if should stop
                front_sensors = [ir[0], ir[7]]
                max_front = max(front_sensors)
                # alot of lenient threshold  - exit should have very low readings
                if not walls.front and max_front < IR_THRESHOLD * 0.4:
                    # At exit - SHOULDSTOP!
                    mp.stop()
                    print(f"\n{'='*60}")
                    print(f"✓ EXIT REACHED at step {steps}!")
                    print(f"Final pose: ({est_x:.3f}, {est_y:.3f}, {math.degrees(est_theta):.1f}°)")
                    print(f"Final cell: {current_cell}")
                    print(f"Goal cell: ({GOAL_ROW}, {GOAL_COL})")
                    print(f"Front sensors: {front_sensors[0]:.1f}, {front_sensors[1]:.1f} (no wall = exit!)")
                    print(f"{'='*60}")
                    break
            
           
            if next_cell != current_cell:
                action = get_action_to_cell(current_cell, next_cell, est_theta)
                
                if action:
                    # LOOP PREVENTION: Check if we're repeating the same action
                    if action == last_action:
                        action_repeat_count += 1
                        if action_repeat_count >= MAX_ACTION_REPEATS:
                            # Stuck in loop - switch to exploration
                            print(f"  [WARN] Stuck in loop, switching to exploration...")
                            action_executed_this_step = False
                            exploration_mode = True
                        else:
                            # Still trying same action, but not too many times yet
                            pass
                    else:
                        # New action - reset counter
                        action_repeat_count = 0
                        last_action = action
                    
                    # Only execute if not stuck in loop
                    if action_repeat_count < MAX_ACTION_REPEATS:
                        # If action is forward and no wall, execute it
                        # Don't let path planning override basic wall detection
                        if action[0] == 'forward':
                            # Check if there's actually a wall before moving
                            if not walls.front:
                                if steps % 5 == 0:  # Print less frequently
                                    print(f"\n[STEP {steps}] Localized: {localized}, Navigating to exit")
                                    print(f"  Pose: ({est_x:.3f}, {est_y:.3f}, {math.degrees(est_theta):.1f}°)")
                                    print(f"  Cell: {current_cell}, Goal: ({GOAL_ROW}, {GOAL_COL})")
                                    print(f"  Path length: {len(path)}, Next: {next_cell}")
                                    print(f"  Action: forward")
                                
                                if execute_action(action, walls, ir):
                                    moved_at_least_once = True
                                    action_executed_this_step = True
                            else:
                                # Wall detected, fall back to exploration
                                print(f"  [WARN] Wall detected, switching to exploration...")
                                action_executed_this_step = False  # Let exploration handle it
                        else:
                            # check if it should go forward instead
                            # If no wall in front, prefer forward over turning
                            if not walls.front:
                                # No wall goes forward instead of turning
                                if steps % 5 == 0:
                                    print(f"\n[STEP {steps}] No wall in front, going forward instead of turning")
                                if execute_action(('forward', 0), walls, ir):
                                    moved_at_least_once = True
                                    action_executed_this_step = True
                                    last_action = ('forward', 0)  
                            else:
                                # Wall in front, turn is necessary
                                if steps % 5 == 0:  # Print less frequently
                                    print(f"\n[STEP {steps}] Localized: {localized}, Navigating to exit")
                                    print(f"  Pose: ({est_x:.3f}, {est_y:.3f}, {math.degrees(est_theta):.1f}°)")
                                    print(f"  Cell: {current_cell}, Goal: ({GOAL_ROW}, {GOAL_COL})")
                                    print(f"  Path length: {len(path)}, Next: {next_cell}")
                                    print(f"  Action: {action[0]} {action[1] if action[0]=='turn' else ''}")
                                
                                if execute_action(action, walls, ir):
                                    moved_at_least_once = True
                                    action_executed_this_step = True
                                
                                # ckeck again after movement - might have passed through exit
                                # recheck sensors after movement
                                ir_after = irs.read()
                                walls_after = wp.classify(ir_after)
                                front_sensors_after = [ir_after[0], ir_after[7]]
                                max_front_after = max(front_sensors_after)
                                
                                # check if we're at an edge cell after movement
                                is_at_edge_after = (
                                    current_cell[0] == 0 or current_cell[0] == MAZE_ROWS - 1 or
                                    current_cell[1] == 0 or current_cell[1] == MAZE_COLS - 1
                                )
                                
                                # f at edge with no wall = exit
                                if is_at_edge_after and not walls_after.front and max_front_after < IR_THRESHOLD * 0.4:
                                    # Just passed through exit - stop
                                    mp.stop()
                                    print(f"\n{'='*60}")
                                    print(f"✓ EXIT REACHED at step {steps}!")
                                    print(f"Final pose: ({est_x:.3f}, {est_y:.3f}, {math.degrees(est_theta):.1f}°)")
                                    print(f"Final cell: {current_cell}")
                                    if goal_detected:
                                        print(f"Goal cell: ({GOAL_ROW}, {GOAL_COL})")
                                    print(f"Front sensors: {front_sensors_after[0]:.1f}, {front_sensors_after[1]:.1f} (no wall = exit!)")
                                    print(f"{'='*60}")
                                    break
            else:
                # path planning failed, fall back to exploration
                # doent print warning every step to reduce spam
                if steps % 10 == 0:
                    print(f"  [WARN] Path planning issue, using exploration...")
                
                action_executed_this_step = False
        else:
            # No path found - check if we're already at goal before warning
            if goal_detected and current_cell == (GOAL_ROW, GOAL_COL):
                # it's at goal check sensors and declare success
                front_sensors_check = [ir[0], ir[7]]
                max_front_check = max(front_sensors_check)
                walls_check = wp.classify(ir)
                
                if not walls_check.front or max_front_check < IR_THRESHOLD * 0.6:
                    # it's at goal and sensors confirm no wall = EXIT REACHED
                    mp.stop()
                    print(f"\n{'='*60}")
                    print(f"✓ GOAL REACHED at step {steps}!")
                    print(f"Final pose: ({est_x:.3f}, {est_y:.3f}, {math.degrees(est_theta):.1f}°)")
                    print(f"Final cell: {current_cell}")
                    print(f"Goal cell: ({GOAL_ROW}, {GOAL_COL})")
                    print(f"Front sensors: {front_sensors_check[0]:.1f}, {front_sensors_check[1]:.1f} (no wall = exit!)")
                    print(f"{'='*60}")
                    break
            else:
                # No path found - use exploration mode
                # doesn't print warning every step
                if steps % 10 == 0:
                    print(f"  [WARN] No path to goal, exploring...")
                # let exploration mode handle it (will be handled below)
                action_executed_this_step = False
    else:
        # Systematic path exploration with backtracking
        exploration_mode = True
        if steps % 20 == 0:  # Print less frequently during exploration
            print(f"\n[STEP {steps}] Exploring (localizing...)")
            print(f"  Estimated cell: {current_cell}, Pose: ({est_x:.3f}, {est_y:.3f})")
        
        # Track visited cells for backtracking
        if last_visited_cell != current_cell:
            # New cell visited - add to current path and exploration stack
            if last_visited_cell is not None:
                if last_visited_cell not in exploration_stack:
                    exploration_stack.append(last_visited_cell)
                current_path_cells.append(last_visited_cell)
            last_visited_cell = current_cell
        
        # Check for exit first (before any movement)
        exit_cell = detect_exit(current_cell[0], current_cell[1], walls, est_theta, ir)
        if exit_cell and not goal_detected:
            GOAL_ROW, GOAL_COL = exit_cell
            planner.set_goal_cells([(GOAL_ROW, GOAL_COL)])
            goal_detected = True
            print(f"\n[EXIT DETECTED] Goal set to cell ({GOAL_ROW}, {GOAL_COL})")
            print(f"  Robot at edge cell, sensors confirm no wall = EXIT!")
        
        # Reset directions tried if we moved to a new cell
        if last_visited_cell != current_cell:
            current_cell_directions_tried = set()
            last_cell_before_turn = None
        
        # Determine forward cell and current direction
        forward_cell = None
        theta_norm = est_theta % (2 * math.pi)
        current_direction = None
        if abs(theta_norm) < 0.2 or abs(theta_norm - 2 * math.pi) < 0.2:  
            forward_cell = (current_cell[0], current_cell[1] + 1)
            current_direction = 'E'
        elif abs(theta_norm - math.pi / 2) < 0.2:  
            forward_cell = (current_cell[0] - 1, current_cell[1])
            current_direction = 'N'
        elif abs(theta_norm - math.pi) < 0.2:  
            forward_cell = (current_cell[0], current_cell[1] - 1)
            current_direction = 'W'
        elif abs(theta_norm + math.pi / 2) < 0.2 or abs(theta_norm - 3 * math.pi / 2) < 0.2:  
            forward_cell = (current_cell[0] + 1, current_cell[1])
            current_direction = 'S'
        
        
        # doesn't check if visited - just keeps going forward until wall blocks it
        if not walls.front and forward_cell and maze_map.in_bounds(forward_cell[0], forward_cell[1]):
            # Check if forward cell is the exit before moving
            if goal_detected and forward_cell == (GOAL_ROW, GOAL_COL):
                # About to move to exit - check sensors
                front_sensors = [ir[0], ir[7]]
                max_front = max(front_sensors)
                if not walls.front and max_front < IR_THRESHOLD * 0.4:
                    # At exit - STOP!
                    mp.stop()
                    print(f"\n{'='*60}")
                    print(f"✓ EXIT REACHED at step {steps}!")
                    print(f"Final pose: ({est_x:.3f}, {est_y:.3f}, {math.degrees(est_theta):.1f}°)")
                    print(f"Final cell: {current_cell}")
                    print(f"Goal cell: ({GOAL_ROW}, {GOAL_COL})")
                    print(f"Front sensors: {front_sensors[0]:.1f}, {front_sensors[1]:.1f} (no wall = exit!)")
                    print(f"{'='*60}")
                    break
            
            # Keeps going forward - doesn't stop just because cell is visited
            move_success = execute_action(('forward', 0), walls, ir)
            if move_success:
                moved_at_least_once = True
                action_executed_this_step = True
                
                # Check if we just moved to exit cell
                # reestimate pose after movement
                est_x_after, est_y_after, est_theta_after = pf.estimate_pose()
                current_cell_after = world_to_cell(est_x_after, est_y_after)
                
                # reread sensors after movement
                ir_after = irs.read()
                walls_after = wp.classify(ir_after)
                front_sensors_after = [ir_after[0], ir_after[7]]
                max_front_after = max(front_sensors_after)
                
                # Check if it's at exit cell after movement
                is_at_edge_after = (
                    current_cell_after[0] == 0 or current_cell_after[0] == MAZE_ROWS - 1 or
                    current_cell_after[1] == 0 or current_cell_after[1] == MAZE_COLS - 1
                )
                
                if (goal_detected and current_cell_after == (GOAL_ROW, GOAL_COL)) or (is_at_edge_after and not walls_after.front and max_front_after < IR_THRESHOLD * 0.4):
                    # Just moved to exit - STOP!
                    mp.stop()
                    print(f"\n{'='*60}")
                    print(f"✓ EXIT REACHED at step {steps}!")
                    print(f"Final pose: ({est_x_after:.3f}, {est_y_after:.3f}, {math.degrees(est_theta_after):.1f}°)")
                    print(f"Final cell: {current_cell_after}")
                    if goal_detected:
                        print(f"Goal cell: ({GOAL_ROW}, {GOAL_COL})")
                    print(f"Front sensors: {front_sensors_after[0]:.1f}, {front_sensors_after[1]:.1f} (no wall = exit!)")
                    print(f"{'='*60}")
                    break

        # Wall detected in front - check sides
        elif walls.front:
            # Wall in front now check right/left
            if not walls.right:
                # Right is open turns right and explores that path
                if execute_action(('turn', -90), walls, ir):
                    moved_at_least_once = True
                    action_executed_this_step = True
                    last_cell_before_turn = current_cell
            elif not walls.left:
                # Left is open turns left and explores that path
                if execute_action(('turn', 90), walls, ir):
                    moved_at_least_once = True
                    action_executed_this_step = True
                    last_cell_before_turn = current_cell
            else:
                # Dead end all directions blocked (front, right, left)
                # This means it's explored the entire path now backtrack
                if exploration_stack and last_cell_before_turn != current_cell:
                    
                    if execute_action(('turn', 90), walls, ir):
                        if execute_action(('turn', 90), walls, ir):
                            moved_at_least_once = True
                            action_executed_this_step = True
                            current_path_cells = []  # Clear path when backtracking
                else:
                    
                    if execute_action(('turn', 90), walls, ir):
                        if execute_action(('turn', 90), walls, ir):
                            moved_at_least_once = True
                            action_executed_this_step = True
    
    # Only force action if it's truly stuck (shouldn't happen with simplified logic)
    # But doesn't force unnecessary turns only if it can't move forward and have no other option
    if not action_executed_this_step and walls.front:
        # Only forces action if front is blocked and it's not done anything
        if not walls.right:
            if execute_action(('turn', -90), walls, ir):
                moved_at_least_once = True
                action_executed_this_step = True
        elif not walls.left:
            if execute_action(('turn', 90), walls, ir):
                moved_at_least_once = True
                action_executed_this_step = True
    
    steps += 1

print(f"\n{'='*60}")
print(f"Controller finished after {steps} steps")
print(f"Final estimated pose: ({est_x:.3f}, {est_y:.3f}, {math.degrees(est_theta):.1f}°)")
print(f"Final cell: {current_cell}")
print(f"{'='*60}")
