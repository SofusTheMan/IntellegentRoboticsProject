import math


class MotionPrimitives:
    """Basic motion for an e-puck-like diff-drive robot in Webots."""
    def __init__(self, robot, left_motor, right_motor,
                 wheel_radius_m=0.0205, axle_length_m=0.052):
        self.robot = robot
        self.dt = int(robot.getBasicTimeStep())
        self.r = wheel_radius_m
        self.L = axle_length_m
        self.left = left_motor
        self.right = right_motor
        self.max_ws = min(self.left.getMaxVelocity(), self.right.getMaxVelocity())
        self.left.setPosition(float('inf'))
        self.right.setPosition(float('inf'))

    def _step_secs(self, secs):
        steps = max(1, int(secs / (self.dt / 1000.0)))
        for _ in range(steps):
            if self.robot.step(self.dt) == -1:
                break

    def stop(self, settle_ms=120):
        self.left.setVelocity(0.0)
        self.right.setVelocity(0.0)
        self._step_secs(settle_ms / 1000.0)

    def turn_deg(self, angle_deg, omega_body=0.7):
        angle_rad = math.radians(angle_deg)
        wl = -(omega_body * self.L) / (2 * self.r)
        wr = +(omega_body * self.L) / (2 * self.r)
        wl = max(min(wl, self.max_ws), -self.max_ws)
        wr = max(min(wr, self.max_ws), -self.max_ws)

        if angle_rad >= 0.0:
            self.left.setVelocity(wl)
            self.right.setVelocity(wr)
        else:
            self.left.setVelocity(-wl)
            self.right.setVelocity(-wr)

        t_needed = abs(angle_rad / omega_body)
        self._step_secs(t_needed)
        self.stop()

    def forward_m(self, distance_m, v_body=0.05):
        wheel_speed = v_body / self.r
        wheel_speed = max(min(wheel_speed, self.max_ws), -self.max_ws)
        sign = 1.0 if distance_m >= 0 else -1.0
        self.left.setVelocity(sign * wheel_speed)
        self.right.setVelocity(sign * wheel_speed)
        t_needed = abs(distance_m / v_body)
        self._step_secs(t_needed)
        self.stop()

    def turn_90_right(self):
        self.turn_deg(-90.0)

    def turn_90_left(self):
        self.turn_deg(+90.0)

    def forward_cells(self, n_cells, cell_size_m, check_sensors=None, stop_on_wall=True):
        """
        Move forward n_cells, checking sensors during movement.
        If check_sensors is provided (function that returns True if wall detected),
        will stop immediately if wall is detected.
        """
        distance_m = n_cells * cell_size_m
        wheel_speed = 0.05 / self.r
        wheel_speed = max(min(wheel_speed, self.max_ws), -self.max_ws)
        
        self.left.setVelocity(wheel_speed)
        self.right.setVelocity(wheel_speed)
        
        # Move incrementally, checking sensors at each step
        t_total = abs(distance_m / 0.05)
        steps_needed = max(1, int(t_total / (self.dt / 1000.0)))
        
        for step in range(steps_needed):
            if self.robot.step(self.dt) == -1:
                break
            
            # Check sensors during movement - stop immediately if wall detected
            if check_sensors is not None and stop_on_wall:
                if check_sensors():
                    # Emergency stop!
                    self.left.setVelocity(0.0)
                    self.right.setVelocity(0.0)
                    self._step_secs(0.1)  # Brief settle
                    return False  # Movement interrupted by wall
        
        self.stop()
        return True
