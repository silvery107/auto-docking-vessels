class PID_Controller:
    def __init__(self, Kp=0.0, Kd=0.0, Ki=0.0):
        self.Kp = float(Kp)
        self.Kd = float(Kd)
        self.Ki = float(Ki)
        self.err_last = 0.0
        self.err_int = 0.0
    
    def apply(self, err):
        err = float(err)
        self.err_int += err
        u = self.Kp * err + self.Kd * (err-self.err_last) + self.Ki * self.err_int
        self.err_last = err
        return u

    def reset(self):
        self.err_int = 0.0
        self.err_last = 0.0
