OBSTACLE_THRESHOLD = 73
RIGHT_SENSOR = [1, 2]  # ps2 - right side sensor
LEFT_SENSOR = [5, 6]  # ps5 - left side sensor
FRONT_SENSORS = [0, 7]  # ps0 and ps7 - front sensors
BACK_SENSORS = [3, 4]  # ps4 and ps1 - back sensors

def get_walls_around_robot(sensors):
    """Return a dict indicating presence of walls around the robot."""
    front_wall = sum(sensors[i].getValue() for i in FRONT_SENSORS) / len(FRONT_SENSORS) > 100
    right_wall = sum(sensors[i].getValue() for i in RIGHT_SENSOR) / len(RIGHT_SENSOR) > 100
    left_wall = sum(sensors[i].getValue() for i in LEFT_SENSOR) / len(LEFT_SENSOR) > 100
    back_wall = sum(sensors[i].getValue() for i in BACK_SENSORS) / len(BACK_SENSORS) > 100
    
    return {
        "front": front_wall,
        "right": right_wall,
        "left": left_wall,
        "back": back_wall
    }