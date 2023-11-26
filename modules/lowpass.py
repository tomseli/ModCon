import numpy as np

class LowPass():
    def __init__(self, tau : float) -> None:
        self.tau = tau
        self.x_filter = 0
        pass

    def filter(self, x, dt) -> float:
        x_d_filter = (x - self.x_filter)/self.tau
        self.x_filter = self.x_filter + x_d_filter * dt
        return self.x_filter