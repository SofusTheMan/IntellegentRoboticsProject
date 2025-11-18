from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Motors
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

forward_speed = 3.0
left_motor.setVelocity(forward_speed)
right_motor.setVelocity(forward_speed)
# left_motor.setVelocity(0)
# right_motor.setVelocity(0)

# IR sensors
sensors = []
for i in range(8):
    name = f'ps{i}'
    sensor = robot.getDevice(name)
    sensor.enable(timestep)
    sensors.append(sensor)


while robot.step(timestep) != -1:

    if sensors[0].getValue() > 90 or sensors[7].getValue() > 90:
        print("Obstacle detected! Turning.")
        left_motor.setVelocity(-forward_speed)
        right_motor.setVelocity(forward_speed)
    elif sensors[1].getValue() > 90 or sensors[6].getValue() > 90:
        print("Obstacle detected! Slight turn.")
        left_motor.setVelocity(0)
        right_motor.setVelocity(forward_speed)
    # if right side is clear, steer right (left wheel faster than right)
    elif sensors[2].getValue() < 90 and sensors[3].getValue() < 90:
        print("Right clear. Steering right.")
        left_motor.setVelocity(forward_speed)
        right_motor.setVelocity(forward_speed * 0.3)
    else:
        left_motor.setVelocity(forward_speed)
        right_motor.setVelocity(forward_speed)
