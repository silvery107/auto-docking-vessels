class Teleop:
    def __init__(self, robot, keyboard) -> None:
        self.key = 0
        self.vx = 0
        self.vy = 0
        self.wz = 0
        self._estop_flagged = False
        self.robot = robot
        self.keyboard = keyboard

    def keyboardControl(self, key):
        self.key = key
        if key == ord('W'):
            self.vx = -0.5
            # self.printX('move forward')
        elif key == ord('S'):
            self.vx = 0.5
            # self.printX('move backward')
        elif key == ord('A'):
            self.vy = 0.5
            # self.printX('move left')
        elif key == ord('D'):
            self.vy = -0.5
            # self.printX('move right')
        elif key == ord('Q'):
            self.wz = 0.5
            # self.printX('turn left')
        elif key == ord('E'):
            self.wz = -0.5
            # self.printX('turn right')
        else:
            self.vx = 0
            self.vy = 0
            self.wz = 0

    def get_command(self):
        self.key = self.keyboard.getKey()
        self.keyboardControl(self.key)
        return [self.vx, self.vy, 0], self.wz, self._estop_flagged
