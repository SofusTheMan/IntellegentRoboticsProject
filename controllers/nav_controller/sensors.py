class IRSuite:
    def __init__(self, robot, names=("ps0", "ps1", "ps2", "ps3",
                                     "ps4", "ps5", "ps6", "ps7"),
                 noise_sigma=0.0):
        self.robot = robot
        self.dt = int(robot.getBasicTimeStep())
        self.noise_sigma = noise_sigma
        self.sensors = []
        for n in names:
            s = robot.getDevice(n)
            s.enable(self.dt)
            self.sensors.append(s)

    def _noisy(self, v):
        import random
        if self.noise_sigma <= 0.0:
            return v
        return v + random.gauss(0, self.noise_sigma)

    def read(self):
        return [self._noisy(s.getValue()) for s in self.sensors]
