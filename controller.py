import numpy as np

class Control():
    def __init__(self) -> None:
        self.kp = 0
        self.ki = 0
        self.kd = 0

        self.i = 0
        self.d = 0

        self.p_out = 0
        self.i_out = 0
        self.d_out = 0
        self.sum = 0

        self.e = 0
        self.e_prev = 0
  
    def control(self, current, setpoint, dt):
        if(dt == 0):
            return 0

        self.e = setpoint - current

        self.i = self.i + self.e * dt
        self.d = (self.e - self.e_prev)/dt

        self.p_out = self.kp * self.e
        self.i_out = self.ki * self.i
        self.d_out = self.kd * self.d

        self.sum = self.p_out + self.i_out + self.d_out
        self.e_prev = self.e

        return self.sum
    
    def print(self):
        print("P {:.4f} \t I {:.4f} \t D {:.4f} \t SUM {:.4f} \t E {:.4f}".format(
            self.p_out, self.i_out, self.d_out, self.sum, self.e
        ))