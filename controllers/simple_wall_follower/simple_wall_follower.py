"""locomotive controller - right-hand wall follower for 0.12m hex maze."""

from controller import Robot, Emitter
import struct
import time
import os
import json

def report_trial_complete(step_count, trial_start_time, success):
    """
    Report trial completion to supervisor.
    Call this once when trial ends (success or failure).
    """
    elapsed_time = robot.getTime() - trial_start_time
    
    script_dir = os.path.dirname(os.path.abspath(__file__))
    signal_file = os.path.join(script_dir, 'trial_complete.json')
    
    data = {
        'completed': success,
        'steps': step_count,
        'time': elapsed_time
    }
    
    with open(signal_file, 'w') as f:
        json.dump(data, f)
    
    print(f"Trial complete: Success={success}, Steps={step_count}, Time={elapsed_time:.2f}s")

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Motors
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))


trial_start_time = robot.getTime()
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

# Emitter
emitter = robot.getDevice('emitter')

# State machine
STATE_FORWARD = 0
STATE_TURN_RIGHT = 1
STATE_TURN_LEFT = 2

state = STATE_FORWARD
state_start_time = -999

# Thresholds
OBSTACLE_THRESHOLD = 90
RIGHT_SENSOR = 2  # ps2 - right side sensor
FRONT_SENSORS = [0, 7]  # ps0 and ps7 - front sensors

def has_wall_right():
    """Check if there's a wall on the right side."""
    return sensors[RIGHT_SENSOR].getValue() > OBSTACLE_THRESHOLD

def has_obstacle_front():
    """Check if there's an obstacle in front."""
    return (sensors[FRONT_SENSORS[0]].getValue() > 75 or 
            sensors[FRONT_SENSORS[1]].getValue() > 75)

def move_forward(distance, speed):
    """Calculate time needed to move forward by distance at given speed."""
    return distance / speed * 48.0

def turn_90_degrees(angular_speed):
    """Calculate time needed for 90-degree turn."""
    return 4.4  # seconds (adjust as needed)
count = 0
while robot.step(timestep) != -1:
    current_time = robot.getTime()
    
    # Broadcast velocities
    message = struct.pack("ff", left_motor.getVelocity(), right_motor.getVelocity())
    emitter.send(message)
    
    # Debug: print sensor values occasionally
    # if int(current_time * 2) % 10 == 0:  # Every 5 seconds
    #     print(f"Right sensor: {sensors[RIGHT_SENSOR].getValue():.1f}, Front: {sensors[0].getValue():.1f}, {sensors[7].getValue():.1f}")
    
    if state == STATE_FORWARD:
        # Move forward for one cell
        left_motor.setVelocity(forward_speed)
        right_motor.setVelocity(forward_speed)
        
        if state_start_time == 0:
            state_start_time = current_time
        
        # After moving 0.12m, decide next action
        if current_time - state_start_time >= move_forward(cell_size, forward_speed):
            count += 1
            print("Number of steps taken:", count)
            state_start_time = 0
            print("Reached next cell, evaluating surroundings")
            # RIGHT-HAND WALL FOLLOWING LOGIC:
            # Priority 1: If NO wall on right, turn right (to find the wall)
            # Print all sensor readings
            readings = ", ".join(f"ps{i}={sensors[i].getValue():.1f}" for i in range(len(sensors)))
            print(f"Sensor readings: {readings}")
            # If no walls detected by any sensor -> finished maze
            if all(s.getValue() < 80 for s in sensors):
                left_motor.setVelocity(0.0)
                right_motor.setVelocity(0.0)
                print(f"Finished maze. Total steps: {count}")
                report_trial_complete(count, trial_start_time, True)
                break
            if not has_wall_right():
                print("No wall on right - turning right to find wall")
                state = STATE_TURN_RIGHT
            # Priority 2: If wall on right AND obstacle in front, turn left
            elif has_obstacle_front():
                print("Wall on right + front blocked - turning left")
                state = STATE_TURN_LEFT
            # Priority 3: Wall on right, front clear - continue forward
            else:
                print("Wall on right, continuing forward")
                state = STATE_FORWARD
    
    elif state == STATE_TURN_RIGHT:
        # Turn right 90 degrees
        left_motor.setVelocity(turn_speed)
        right_motor.setVelocity(-turn_speed)
        
        if state_start_time == 0:
            state_start_time = current_time
        
        if current_time - state_start_time >= turn_90_degrees(turn_speed):
            state_start_time = 0
            count += 1
            print("Number of steps taken:", count)
            state = STATE_FORWARD
            print("Turn right complete")
    
    elif state == STATE_TURN_LEFT:
        # Turn left 90 degrees
        left_motor.setVelocity(-turn_speed)
        right_motor.setVelocity(turn_speed)
        
        if state_start_time == 0:
            state_start_time = current_time
        
        if current_time - state_start_time >= turn_90_degrees(turn_speed):
            state_start_time = -999
            state = STATE_FORWARD
            print("Turn left complete")