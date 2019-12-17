import numpy as np
class Controller:
    def __init__(self, Kp, Ki, Kd, Ts=0.1):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.Ts = Ts
        self.integral = 0
        self.derivative = 0
        self.old_yaw = 0

    def update(self, yaw):
        self.integral = self.integral + yaw * self.Ts
        self.derivative = (yaw-self.old_yaw) / self.Ts
        P = self.Kp * yaw
        I = self.Ki * self.integral
        D = self.Kd * self.derivative
        output = P + I + D

        vel = -1*np.tanh(output)

        self.old_yaw = yaw

        return vel
