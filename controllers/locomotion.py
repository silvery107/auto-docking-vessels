

class Locomotion:
    def __init__(self, robot_, motors_) -> None:
        self.robot = robot_
        self.motors = motors_
        self.MAX_VEL = 0.5
        # self.speeds = []
        # for motor in self.motors:
        #     self.speeds.append(motor.getVelocity())
    
    def limiting(self, vel):
        if vel>self.MAX_VEL:
            return self.MAX_VEL
        elif vel<-self.MAX_VEL:
            return -self.MAX_VEL
        else:
            return vel

    def forward(self, vx):
        self.motors[2].setVelocity(self.limiting(vx))
        self.motors[3].setVelocity(self.limiting(vx))

    def backward(self, vx):
        self.forward(-vx)
    
    def turnRight(self, wz):
        speed0 = self.motors[0].getVelocity()
        speed1 = self.motors[1].getVelocity()
        self.motors[0].setVelocity(self.limiting(speed0+wz))
        self.motors[1].setVelocity(self.limiting(speed1-wz))

    def turnLeft(self, wz):
        self.turnRight(-wz)

    def moveLeft(self, vy):
        self.motors[0].setVelocity(self.limiting(vy))
        self.motors[1].setVelocity(self.limiting(vy))

    def moveRight(self, vy):
        self.moveLeft(-vy)
        