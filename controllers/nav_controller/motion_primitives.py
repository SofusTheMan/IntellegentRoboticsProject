import math

class MotionPrimitives:
    """
    Basic motion for an e-puck-like diff-drive robot in Webots.
    Uses time-based actuation with modest speeds (safe to start).
    Later you can refine with encoders/IMU for tighter accuracy.
    """
    def __init__(self, robot, left_motor, right_motor,
                 wheel_radius_m=0.0205, axle_length_m=0.052):
        self.robot = robot
        self.dt = int(robot.getBasicTimeStep())
        self.r = wheel_radius_m
        self.L = axle_length_m
        self.left = left_motor
        self.right = right_motor
        self.max_ws = min(self.left.getMaxVelocity(), self.right.getMaxVelocity())
        # velocity mode
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
        """
        Turn in-place by +angle_deg (CCW) or -angle_deg (CW).
        omega_body: body angular rate [rad/s] (0.5–1.0 is safe)
        """
        angle_rad = math.radians(angle_deg)
        # in-place: wr = -wl = omega_body * L/(2*r)
        wl = -(omega_body * self.L) / (2 * self.r)
        wr = +(omega_body * self.L) / (2 * self.r)
        wl = max(min(wl, self.max_ws), -self.max_ws)
        wr = max(min(wr, self.max_ws), -self.max_ws)
        self.left.setVelocity(wl if angle_rad >= 0 else -wl)
        self.right.setVelocity(wr if angle_rad >= 0 else -wr)
        t_needed = abs(angle_rad / omega_body)
        self._step_secs(t_needed)
        self.stop()

    def forward_m(self, distance_m, v_body=0.05):
        """
        Drive forward (or backward if negative) by distance_m meters.
        v_body: linear speed [m/s] (0.04–0.07 is typical)
        """
        wheel_speed = v_body / self.r
        wheel_speed = max(min(wheel_speed, self.max_ws), -self.max_ws)
        sgn = 1.0 if distance_m >= 0 else -1.0
        self.left.setVelocity(sgn * wheel_speed)
        self.right.setVelocity(sgn * wheel_speed)
        t_needed = abs(distance_m / v_body)
        self._step_secs(t_needed)
        self.stop()

    # helpers Sofus suggested
    def turn_90_right(self):  # CW
        self.turn_deg(-90.0)

    def turn_90_left(self):   # CCW
        self.turn_deg(+90.0)

    def forward_cells(self, n_cells, cell_size_m):
        self.forward_m(n_cells * cell_size_m)
