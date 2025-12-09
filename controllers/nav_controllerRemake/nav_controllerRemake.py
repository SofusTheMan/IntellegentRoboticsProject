"""locomotive controller - right-hand wall follower for 0.12m hex maze."""

from controller import Robot, Emitter
import struct
import time
from maze_graph_adapter import load_maze_graph_from_file, NORTH, EAST, SOUTH, WEST, MazeGraphAdapter
import os
import json
import orientation
import update_possible_positions
from make_action import make_action
from next_action import get_next_action

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Motors
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# Movement parameters
forward_speed = 1.0
turn_speed = 0.5
cell_size = 0.1  # meters per hex cell

# IR sensors
sensors = []
for i in range(8):
    name = f'ps{i}'
    sensor = robot.getDevice(name)
    sensor.enable(timestep)
    sensors.append(sensor)

def load_maze_graph_from_file(filepath=None):
    """Load MazeGraph from JSON file."""
    if filepath is None:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        filepath = os.path.join(script_dir, 'maze_graph.json')
    
    if not os.path.exists(filepath):
        raise FileNotFoundError(f"MazeGraph file not found: {filepath}")
    
    with open(filepath, 'r') as f:
        data = json.load(f)
    
    return MazeGraphAdapter(data)

# Load MazeGraph
try:
    maze_map = load_maze_graph_from_file()
    MAZE_ROWS = maze_map.rows
    MAZE_COLS = maze_map.cols
    MAZE_SIDE_LENGTH = maze_map.side_length
    WALL_CELL_RATIO = maze_map.wall_cell_ratio
    print(f"Loaded MazeGraph: {MAZE_ROWS}x{MAZE_COLS}, side_length={MAZE_SIDE_LENGTH}")
except FileNotFoundError as e:
    print(f"ERROR: {e}")
    exit(1)

# State machine
STATE_IDLE = 0           # Deciding what to do
STATE_EXECUTING = 1      # Executing an action

# Action types
ACTION_FORWARD = 0
ACTION_TURN_RIGHT = 1
ACTION_TURN_LEFT = 2

state = STATE_IDLE
current_action = None
action_start_time = 0

# Thresholds
OBSTACLE_THRESHOLD = 90
RIGHT_SENSOR = [1, 2]  # ps2 - right side sensor
LEFT_SENSOR = [5, 6]  # ps5 - left side sensor
FRONT_SENSORS = [0, 7]  # ps0 and ps7 - front sensors
BACK_SENSORS = [3, 4]  # ps4 and ps1 - back sensors

# Initialize possible positions
possible_positions = []
for i in range(maze_map.rows):
    for j in range(maze_map.cols):
        for direction in [NORTH, EAST, SOUTH, WEST]:
            possible_positions.append((i, j, direction))

step_count = 0

def observe_and_filter(sensors, possible_positions, maze_map):
    """Observe walls and filter possible positions."""
    readings = ", ".join(f"ps{i}={sensors[i].getValue():.1f}" for i in range(len(sensors)))
    print(f"Sensor readings: {readings}")
    
    walls_around = orientation.get_walls_around_robot(sensors)
    print(f"Walls around robot: {walls_around}")
    print(f"Possible positions before filter: {possible_positions}")
    
    filtered = update_possible_positions.update_possible_positions(
        walls_around, possible_positions, maze_map)
    
    print(f"Possible positions after filter: {filtered}")
    if len(filtered) <= 5:
        for pos in filtered:
            print(f"  Position: row={pos[0]}, col={pos[1]}, dir={pos[2]}")
    
    return filtered

def has_wall_right():
    """Check if there's a wall on the right side."""
    print("Sensor readings for right wall check:", ", ".join(f"ps{i}={sensors[i].getValue():.1f}" for i in RIGHT_SENSOR))
    return any(sensors[i].getValue() > OBSTACLE_THRESHOLD for i in RIGHT_SENSOR)

def has_obstacle_front():
    """Check if there's an obstacle in front."""
    return (sensors[FRONT_SENSORS[0]].getValue() > 75 or 
            sensors[FRONT_SENSORS[1]].getValue() > 75)

def decide_next_action():
    """Right-hand wall following logic."""
    if all(s.getValue() < 80 for s in sensors):
        return None  # Exit maze
    
    if not has_wall_right():
        print("No wall on right - turning right to find wall")
        return ACTION_TURN_RIGHT
    elif has_obstacle_front():
        print("Wall on right + front blocked - turning left")
        return ACTION_TURN_LEFT
    else:
        print("Wall on right, continuing forward")
        return ACTION_FORWARD

def move_forward_time(distance, speed):
    """Calculate time needed to move forward."""
    return distance / speed * 45

def turn_90_time():
    """Calculate time needed for 90-degree turn."""
    return 4.5

# Main control loop
while robot.step(timestep) != -1:
    current_time = robot.getTime()
    
    if state == STATE_IDLE:
        # We just finished an action (or starting fresh)
        # Decide what to do next
        print(f"\n=== STEP {step_count} ===")
        print("State: IDLE - Deciding next action")
        
        # next_action = decide_next_action()
        actions = []
        for pos in possible_positions:
            print(f"  Possible position: row={pos[0]}, col={pos[1]}, dir={pos[2]}")
            actions.append(get_next_action(maze_map, pos, True))
        next_action = max(set(actions), key=actions.count)  # Majority vote
        
        if next_action is None:
            # Finished maze
            left_motor.setVelocity(0.0)
            right_motor.setVelocity(0.0)
            print(f"Finished maze. Total steps: {step_count}")
            break
        
        # Start executing the action
        current_action = next_action
        action_start_time = current_time
        state = STATE_EXECUTING
        
        # Start the motors
        if current_action == ACTION_FORWARD:
            print("Starting: FORWARD")
            left_motor.setVelocity(forward_speed)
            right_motor.setVelocity(forward_speed)
        elif current_action == ACTION_TURN_RIGHT:
            print("Starting: TURN RIGHT")
            left_motor.setVelocity(turn_speed)
            right_motor.setVelocity(-turn_speed)
        elif current_action == ACTION_TURN_LEFT:
            print("Starting: TURN LEFT")
            left_motor.setVelocity(-turn_speed)
            right_motor.setVelocity(turn_speed)
    
    elif state == STATE_EXECUTING:
        # Check if action is complete
        action_complete = False
        
        if current_action == ACTION_FORWARD:
            if current_time - action_start_time >= move_forward_time(cell_size, forward_speed):
                action_complete = True
                print("Completed: FORWARD")
        elif current_action == ACTION_TURN_RIGHT:
            if current_time - action_start_time >= turn_90_time():
                action_complete = True
                print("Completed: TURN RIGHT")
        elif current_action == ACTION_TURN_LEFT:
            if current_time - action_start_time >= turn_90_time():
                action_complete = True
                print("Completed: TURN LEFT")

        
        if action_complete:
            # Stop motors
            left_motor.setVelocity(0.0)
            right_motor.setVelocity(0.0)
            
            step_count += 1
            
            # Update belief state
            print(f"Updating belief state after action {current_action}")
            
            # 1. Apply movement model
            possible_positions = make_action(current_action, possible_positions, maze_map.cols)
            
            # 2. Apply observation model
            possible_positions = observe_and_filter(sensors, possible_positions, maze_map)
            
            if len(possible_positions) == 0:
                print("ERROR: No valid positions remaining! Localization failed.")
                break
            
            # Go back to IDLE to decide next action
            state = STATE_IDLE
            current_action = None