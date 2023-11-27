import numpy as np
import sys

class PhysicsBase():
    def __init__(self) -> None:
        pass

    def step(self) -> None:
        pass


class PhysicsCart(PhysicsBase):
    def __init__(self) -> None:
        # Properties
        self.m = 1
        self.cf = 1

        # Motion
        self.x_dd = 0
        self.x_d = 0
        self.x = 0

        # Input
        self.f = 0

        # Limits
        self.x_min = -sys.maxsize
        self.x_max = +sys.maxsize

    def addPendulum(self, pendulum) -> None:
        self.pendulum = pendulum
        return None
    
    def step(self, dt : float) -> float:
        term1 = -self.pendulum.m * self.pendulum.l \
            * self.pendulum.theta_dd * np.cos(self.pendulum.theta)
        term2 = self.pendulum.m * self.pendulum.l \
            * self.pendulum.theta*np.sin(self.pendulum.theta)
        
        self.x_dd = (term1 + term2) / (self.pendulum.m + self.m) \
            + self.f - self.x_d * self.cf
        self.x_d = self.x_d + self.x_dd * dt
        self.x = self.x + self.x_d * dt

        if(self.x > self.x_max or self.x < self.x_min):
            self.x_dd = 0
            self.x_d = 0
        if(self.x > self.x_max):
            self.x = self.x_max
        if(self.x < self.x_min):
            self.x = self.x_min
        
        return self.x
    
    def set_limit(self, x_offset) -> None:
        self.x_min = -x_offset
        self.x_max = x_offset
        
class PhysicsCartPendulum(PhysicsBase):
    def __init__(self, cart : PhysicsCart) -> None:
        # Input
        self.cart = cart
        
        # Properties
        self.g = -9.81
        self.l = 0
        self.m = 0
        self.cf = 0 # temporary approx. to dampening
        
        # Motion
        self.theta = 0
        self.theta_d = 0
        self.theta_dd = 0
    
    def step(self, dt : float) -> None:
        term1 = -self.cart.x_dd * np.cos(self.theta)
        term2 = self.cart.x_d * np.sin(self.theta)
        term3 = -self.cart.x_d * self.theta_d * np.sin(self.theta)
        term4 = self.g * np.sin(self.theta)
        term5 = self.cf * self.theta_d # temporary approx. to dampening
        numerator = term1 + term2 + term3 + term4 + term5

        self.theta_dd = numerator/-self.l
        self.theta_d = self.theta_d + self.theta_dd * dt
        self.theta = self.theta + self.theta_d * dt

        return -self.theta