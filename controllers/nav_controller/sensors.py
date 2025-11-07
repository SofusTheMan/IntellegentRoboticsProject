
class IRSuite:
    def __init__(self, robot, names=("ps0","ps1","ps2","ps3","ps4","ps5","ps6","ps7")):
        self.robot = robot
        self.dt = int(robot.getBasicTimeStep())
        self.sensors = []
        for n in names:
            s = robot.getDevice(n)
            s.enable(self.dt)
            self.sensors.append(s)
    def read(self):
        return [s.getValue() for s in self.sensors]
