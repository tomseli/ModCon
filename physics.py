import numpy as np
import math
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
    
    def step(self, dt : float, k4_theta1_dd, k4_theta2_dd) -> float:
        term1 = -self.pendulum.m2 * self.pendulum.l1     * k4_theta1_dd                  * np.cos(self.pendulum.theta1)
        term2 = self.pendulum.m2 * self.pendulum.l1    * self.pendulum.theta1_d**2     * np.sin(self.pendulum.theta1)
        term3 = -self.pendulum.m2 * self.pendulum.l2     * k4_theta2_dd                  * np.cos(self.pendulum.theta2)
        term4 = self.pendulum.m2 * self.pendulum.l2    * self.pendulum.theta2_d**2     * np.sin(self.pendulum.theta2)
        
        self.x_dd = (term1 + term2 + term3 + term4) / -(self.m + self.pendulum.m1) \
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
        
        

class DoublePendulum:
    def __init__(self, cart : PhysicsCart):
        self.cart = cart
        
        self.x = 0
        self.m1 = 0
        self.m2 = 0
        self.l1 = 0
        self.l2 = 0
        self.g = 9.81
        self.theta1 = 0
        self.theta2 = 0
        self.theta1_d = 0
        self.theta2_d = 0

    def calculate_theta1_dd(self, k_theta2_dd):
        term1 = self.m2 * self.l2 * k_theta2_dd * math.cos(self.theta1 - self.theta2)
        term2 = self.m2 * self.l2 * self.theta2_d ** 2 * math.sin(self.theta1 - self.theta2)
        term3 = -self.m2 * self.cart.x_dd * math.cos(self.theta1)
        term4 = (self.m1 + self.m2) *self.g  *  math.sin(self.theta1)
        term5 = self.cf * self.theta1_d
    
        denominator = -(self.m1 + self.m2) * self.l1
        return (term1 + term2 + term3 + term4 + term5) / denominator

    def calculate_theta2_dd(self, k_theta1_dd):
        term1 = self.m2*self.l1*k_theta1_dd*math.cos(self.theta1-self.theta2)
        term2 = -(self.m2*self.l1*self.theta1_d**2*math.sin(self.theta1-self.theta2))
        term3 = -(self.m2 * self.cart.x_dd * math.cos(self.theta2))
        term4 = self.m2*self.g*math.sin(self.theta2)
        term5 = self.cf2 * self.theta2_d
        
        denominator = -self.m2*self.l2
        return (term1 + term2 + term3 + term4 + term5) / denominator

    def step(self, dt):   
        k1_theta1_d = self.theta1_d
        k1_theta2_d = self.theta2_d
        k1_theta1_dd = self.calculate_theta1_dd(0)  # Pass 0 for now, as k2_theta2_dd is not computed yet
        k1_theta2_dd = self.calculate_theta2_dd(k1_theta1_dd)

        self.theta1 += dt / 2 * k1_theta1_d
        self.theta2 += dt / 2 * k1_theta2_d
        self.theta1_d += dt / 2 * k1_theta1_dd
        self.theta2_d += dt / 2 * k1_theta2_dd

        k2_theta1_d = self.theta1_d
        k2_theta2_d = self.theta2_d
        k2_theta1_dd = self.calculate_theta1_dd(k1_theta2_dd)
        k2_theta2_dd = self.calculate_theta2_dd(k2_theta1_dd)

        self.theta1 += dt / 2 * k2_theta1_d
        self.theta2 += dt / 2 * k2_theta2_d
        self.theta1_d += dt / 2 * k2_theta1_dd
        self.theta2_d += dt / 2 * k2_theta2_dd

        k3_theta1_d = self.theta1_d
        k3_theta2_d = self.theta2_d
        k3_theta1_dd = self.calculate_theta1_dd(k2_theta2_dd)
        k3_theta2_dd = self.calculate_theta2_dd(k3_theta1_dd)

        self.theta1 += dt * k3_theta1_d
        self.theta2 += dt * k3_theta2_d
        self.theta1_d += dt * k3_theta1_dd
        self.theta2_d += dt * k3_theta2_dd

        k4_theta1_d = self.theta1_d
        k4_theta2_d = self.theta2_d
        k4_theta1_dd = self.calculate_theta1_dd(k3_theta2_dd)
        k4_theta2_dd = self.calculate_theta2_dd(k4_theta1_dd)

        self.theta1 += dt / 6 * (k1_theta1_d + 2 * k2_theta1_d + 2 * k3_theta1_d + k4_theta1_d)
        self.theta2 += dt / 6 * (k1_theta2_d + 2 * k2_theta2_d + 2 * k3_theta2_d + k4_theta2_d)
        self.theta1_d += dt / 6 * (k1_theta1_dd + 2 * k2_theta1_dd + 2 * k3_theta1_dd + k4_theta1_dd)
        self.theta2_d += dt / 6 * (k1_theta2_dd + 2 * k2_theta2_dd + 2 * k3_theta2_dd + k4_theta2_dd)
        self.x = self.cart.step(dt, k4_theta1_dd, k4_theta2_dd)
# =============================================================================
#         self.x = 0
# =============================================================================

        return self.theta1, self.theta2, self.x



           
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
# =============================================================================
#         term1 = -self.cart.x_dd * np.cos(self.theta)
#         term2 = self.cart.x_d * np.sin(self.theta)
#         term3 = -self.cart.x_d * self.theta_d * np.sin(self.theta)
#         term4 = self.g * np.sin(self.theta)
#         term5 = self.cf * self.theta_d # temporary
#         numerator = term1 + term2 + term3 + term4 + term5
# 
#         self.theta_dd = numerator/-self.l
#         self.theta_d = self.theta_d + self.theta_dd * dt
#         self.theta = self.theta + self.theta_d * dt
# 
#         return self.theta
# =============================================================================


# =============================================================================
#         term1 = self.l2*self.theta_dd2*np.cos(self.theta - self.theta2)
#         term2 = -self.l2*self.theta2*np.sin(self.theta - self.theta2)
#         term3 = -self.cart.x_dd*np.sin(self.theta)
#         term4 = self.theta_d2*np.sin(self.theta - self.theta2)
#         term5 = -self.cart.x_d * self.l*np.sin(self.theta)
#         term6 = self.g * self.l * np.sin(self.theta)
#         numerator = term1 + term2 + term3 + term4 + term5 + term6
# 
#         self.theta_dd = numerator/self.l
#         self.theta_d = self.theta_d + self.theta_dd * dt
#         self.theta = self.theta + self.theta_d * dt
# 
#         return self.theta
# =============================================================================
    
    
# =============================================================================
#         term1 = self.l2*self.theta_dd2*np.cos(self.theta - self.theta2)
#         term2 = -self.l2*self.theta2*np.sin(self.theta - self.theta2)
#         term4 = self.theta_d2*np.sin(self.theta - self.theta2)
#         term6 = self.g * self.l * np.sin(self.theta)
#         numerator = term1 + term2 + term4 + term6
# 
#         self.theta_dd = numerator/self.l
#         self.theta_d = self.theta_d + self.theta_dd * dt
#         self.theta = self.theta + self.theta_d * dt
# 
#         return self.theta2
# =============================================================================
    
        
# =============================================================================
#     def stepPend1(self, dt : float) -> None:
#         term1 = self.l2*self.theta_dd2*np.cos(self.theta - self.theta2)
#         term2 = -self.l2*self.theta_d2*np.sin(self.theta - self.theta2)*(self.theta_d - self.theta_d2) 
#         term3 = self.cart.x_d*np.cos(self.theta)
#         term4 = -self.cart.x_d*np.sin(self.theta)*self.theta_d
#         term5 = self.l2*self.theta_d*self.theta_d2*np.sin(self.theta - self.theta2)
#         term6 = -self.cart.x_d*self.theta_d*np.sin(self.theta)
#         term7 = self.g*np.sin(self.theta)
#         term8 = self.m*self.g*np.sin(self.theta)
#         term9 = self.cf * self.theta_d # temporary
#         
#         numerator1 = term1 + term2 + term3 + term4 + term5 +term6
#         deel1 = numerator1 / -self.m
#         
#         deel2 = term7/-self.m2
#         deel3 = term8/ -(self.m + self.m2)*self.l
# 
#         self.theta_dd = deel1 + deel2 + deel3 + term9
#         self.theta_d = self.theta_d + self.theta_dd * dt
#         self.theta = self.theta + self.theta_d * dt
# 
#         return self.theta
#     
#     
#     def stepPend2(self, dt : float) -> None:
#         term1 = self.l2*self.theta_dd2*np.cos(self.theta - self.theta2)
#         term2 = -self.l2*self.theta_d2*np.sin(self.theta - self.theta2)*(self.theta_d - self.theta_d2) 
#         term3 = self.cart.x_d*np.cos(self.theta)
#         term4 = -self.cart.x_d*np.sin(self.theta)*self.theta_d
#         term5 = self.l2*self.theta_d*self.theta_d2*np.sin(self.theta - self.theta2)
#         term6 = -self.cart.x_d*self.theta_d*np.sin(self.theta)
#         term7 = self.g*np.sin(self.theta)
#         term8 = self.m*self.g*np.sin(self.theta)
#         term9 = self.cf * self.theta_d # temporary
#         
#         numerator1 = term1 + term2 + term3 + term4 + term5 +term6
#         deel1 = numerator1 / -self.m
#         
#         deel2 = term7/-self.m2
#         deel3 = term8/ -(self.m + self.m2)*self.l
# 
#         self.theta_dd = deel1 + deel2 + deel3 + term9
#         self.theta_d = self.theta_d + self.theta_dd * dt
#         self.theta = self.theta + self.theta_d * dt
# 
#         return self.theta
# =============================================================================
    
# =============================================================================
#         def stepPend1(self, dt : float) -> None:
#             term1 = -self.cart.x_dd * np.cos(self.theta)
#             term2 = self.cart.x_d * np.sin(self.theta)
#             term3 = -self.cart.x_d * self.theta_d * np.sin(self.theta)
#             term4 = self.g * np.sin(self.theta)
#             term5 = self.cf * self.theta_d # temporary approx. to dampening
#             numerator = term1 + term2 + term3 + term4 + term5
# 
#             self.theta_dd = numerator/-self.l
#             self.theta_d = self.theta_d + self.theta_dd * dt
#             self.theta = self.theta + self.theta_d * dt
# 
#             return self.theta
#         
#         
#         def stepPend2(self, dt : float) -> None:
#             self.theta2 = np.pi
#             return self.theta2
# =============================================================================

# =============================================================================
# 
#         term1 = -self.pendulum.m * self.pendulum.l \
#             * self.pendulum.theta_dd * np.cos(self.pendulum.theta)
#         term2 = self.pendulum.m * self.pendulum.l \
#             * self.pendulum.theta*np.sin(self.pendulum.theta)
#         
#         self.x_dd = (term1 + term2) / (self.pendulum.m + self.m) \
#             + self.f - self.x_d * self.cf
#         self.x_d = self.x_d + self.x_dd * dt
#         self.x = self.x + self.x_d * dt
# 
#         if(self.x > self.x_max or self.x < self.x_min):
#             self.x_dd = 0
#             self.x_d = 0
#         if(self.x > self.x_max):
#             self.x = self.x_max
#         if(self.x < self.x_min):
#             self.x = self.x_min
#         
#         return self.x
# =============================================================================
    
    
    
    
# =============================================================================
#         term1 = self.l*self.l2*self.theta_dd*np.cos(self.theta-self.theta2)
#         term2 = -self.l*self.l2*self.theta_d*np.sin(self.theta - self.theta2)*(self.theta_d - self.theta_d2)
#         term3 = self.cart.x_dd*self.l2*np.cos(self.theta2)
#         term4 = -self.cart.x_d*self.l2*np.sin(self.theta2)*self.theta_d2
#         term5 = self.l*self.l2*self.theta_d*self.theta_d2*np.sin(self.theta - self.theta2)
#         term6 = -self.cart.x_d*self.theta_d2*self.l2*np.sin(self.theta2)
#         term7 = (self.m * self.g * self.l2 * np.sin(self.theta2))
# 
#         
#         numerator1 = term1 + term2 + term3 + term4 + term5 +term6
#         deel1 = (numerator1/ ((self.l2)**2) )
#         deel2 = term7/(((self.l2)**2)* self.m2)
#         
#         
#         self.theta_dd2 = deel1 + deel2
#         self.theta_d2 = self.theta_d2 + self.theta_dd2 * dt
#         self.theta2 = self.theta2 + self.theta_d2 * dt
#         
#         return self.theta2
# 
# =============================================================================
    
    
    
# =============================================================================
#     def stepPend1(self, dt : float) -> float:
#         term1 = self.l2*self.theta_dd2*np.cos(self.theta - self.theta2)
#         term2 = -self.l2*self.theta_d2*np.sin(self.theta - self.theta2)*(self.theta_d - self.theta_d2)
#         term3 = -self.cart.x_dd*np.cos(self.theta)
#         term4 = self.cart.x_d*np.sin(self.theta)*self.theta_d
#         term5 = self.l2*self.theta_d*self.theta_d2*np.sin(self.theta - self.theta2)
#         term6 = -self.cart.x_d*self.theta_d*np.sin(self.theta)
#         term8 = self.m*self.g*np.sin(self.theta)
#         
#         numerator1 = term1 + term2 + term3 + term4 + term5 + term6 + term8
#         deel1 = numerator1 / -self.m
# 
#         self.theta_dd = deel1
#         self.theta_d = self.theta_d + self.theta_dd * dt
#         self.theta = self.theta + self.theta_d * dt
#         
#         print(self.theta_d - self.theta_d2)
#         return self.theta
#     
#     
#     def stepPend2(self, dt : float) -> float:
#         term1 = self.m2*self.l*self.theta_dd* np.cos(self.theta - self.theta2)
#         term2 = -self.m2*self.l*self.theta_d* np.sin(self.theta - self.theta2)* (self.theta_d - self.theta_d2)
#         term3 = -self.m2*self.cart.x_dd*np.cos(self.theta2)
#         term4 = self.m2*self.cart.x_d*np.sin(self.theta2)*self.theta_d2
#         term5 = self.m2*self.l*self.theta_d*self.theta_d2* np.sin(self.theta - self.theta2)
#         term6 = -self.m2*self.cart.x_d*self.theta_d2*np.sin(self.theta2)
#         term8 = self.m*self.g*np.sin(self.theta2)
#         
#         numerator1 = term1 + term2 + term3 + term4 + term5 + term6+ term8
#         deel1 = numerator1 /-(self.l2 * self.m2)
# 
#         self.theta_dd2 = deel1 
#         self.theta_d2 = self.theta_d2 + self.theta_dd2 * dt
#         self.theta2 = self.theta2 + self.theta_d2 * dt
#         
#         return self.theta2
# =============================================================================
    
  
    
# =============================================================================
#     def stepPend1(self, dt : float) -> float:
#         term1 = self.l2*self.theta_dd2*np.cos(self.theta - self.theta2)
#         term2 = -(self.l2*self.theta_d2**2*np.sin(self.theta - self.theta2))
#         term3 = -(self.cart.x_dd*np.cos(self.theta))
#         term4 = self.cart.x_d*np.sin(self.theta)*self.theta_d
#         term5 = self.l2*self.theta_d*self.theta_d2*np.sin(self.theta - self.theta2)
#         term6 = -(self.cart.x_d*self.theta_d*np.sin(self.theta))
#         term8 = self.g*np.sin(self.theta)
#         term9 = self.cf * self.theta_d
#         
#         numerator1 = term1 + term2 + term3 + term4 + term6 + term9 
#         deel2 = term8/ -self.l
#         deel1 = numerator1 / -self.m*self.l
# 
#         self.theta_dd = deel1 + deel2
#         self.theta_d = self.theta_d + self.theta_dd * dt
#         self.theta = self.theta + self.theta_d * dt
#     
#         return self.theta
#     
#     
#     def stepPend2(self, dt : float) -> float:
#         term1 = self.l*self.theta_dd* np.cos(self.theta - self.theta2)
#         term2 = -(self.l*self.theta_d**2* np.sin(self.theta - self.theta2))
#         term3 = -(self.cart.x_dd*np.cos(self.theta2))
#         term4 = self.cart.x_d*np.sin(self.theta2)*self.theta_d2
#         term5 = self.l*self.theta_d*self.theta_d2* np.sin(self.theta - self.theta2)
#         term6 = -(self.cart.x_d*self.theta_d2*np.sin(self.theta2))
#         term8 = self.g*np.sin(self.theta2)
#         term9 = self.cf2*self.theta_d2
#         
#         numerator1 = term1 + term2 + term3 + term4  + term6+ term8 + term9
#         deel1 = -numerator1 /self.l2
# 
#         self.theta_dd2 = deel1 
#         self.theta_d2 = self.theta_d2 + self.theta_dd2 * dt
#         self.theta2 = self.theta2 + self.theta_d2 * dt
#         
#         return self.theta2
# =============================================================================
    
    
    
    
    
    
      
# =============================================================================
#     def runge_kutta(self, func, dt):
#         k1 = func * dt
#         k2 = (func + k1 / 2) * dt
#         k3 = (func + k2 / 2) * dt
#         k4 = (func + k3) * dt
#         return (k1 + 2*k2 + 2*k3 + k4) / 6
#     
#     def stepPend1(self, dt : float) -> float:
#         term1 = self.m2 *self.l2*self.l*self.theta_dd2*np.cos(self.theta - self.theta2)
#         term2 = -(self.m2 * self.l * self.l2*self.theta_d2**2*np.sin(self.theta - self.theta2))
#         term3 = -self.l * self.m2 *self.cart.x_dd*np.cos(self.theta)
#         term4 = self.l * self.m2 *self.cart.x_d*np.sin(self.theta)*self.theta_d
#         term5 = self.l2 * self.l * self.m2 *self.theta_d*self.theta_d2*np.sin(self.theta - self.theta2)
#         term6 = -self.l * self.m2 *self.cart.x_d*self.theta_d*np.sin(self.theta)
#         term8 = self.l * (self.m2 +self.m)*self.g*np.sin(self.theta)
#         term9 = self.cf * self.theta_d
#         
#         numerator1 = term1 + term2 + term3 + term4 + term5 + term6 + term8 + term9
#         deel1 = numerator1 / -((self.m + self.m2)*self.l**2)
# 
#         self.theta_dd = self.runge_kutta(deel1, dt)
#         self.theta_d = self.theta_d + self.theta_dd * dt
#         self.theta = self.theta + self.theta_d * dt
#         
#         return self.theta
#     
#     
#     def stepPend2(self, dt : float) -> float:
#         term1 = self.m2*self.l2*self.l*self.theta_dd* np.cos(self.theta - self.theta2)
#         term2 = -(self.l2*self.m2*self.l*self.theta_d**2* np.sin(self.theta - self.theta2))
#         term3 = -self.l2*self.m2*self.cart.x_dd*np.cos(self.theta2)
#         term4 =self.l2* self.m2*self.cart.x_d*np.sin(self.theta2)*self.theta_d2
#         term5 = self.l2*self.m2*self.l*self.theta_d*self.theta_d2* np.sin(self.theta - self.theta2)
#         term6 = -self.l2*self.m2*self.cart.x_d*self.theta_d2*np.sin(self.theta2)
#         term8 = self.l2*self.m*self.g*np.sin(self.theta2)
#         term9 = self.cf2 * self.theta_d2
#         
#         numerator1 = term1 + term2 + term3 + term4 + term5 + term6+ term8 + term9
#         deel1 = numerator1 /-(self.l2**2 * self.m2)
# 
#         self.theta_dd2 = self.runge_kutta(deel1, dt)
#         self.theta_d2 = self.theta_d2 + self.theta_dd2 * dt
#         self.theta2 = self.theta2 + self.theta_d2 * dt
#         
#         return self.theta2 
# =============================================================================



# =============================================================================
# class DoublePendulum:
#     def __init__(self):
#         self.m1 = 0
#         self.m2 = 0
#         self.l1 = 0
#         self.l2 = 0
#         self.g = 9.81
#         self.theta1 = 0
#         self.theta2 = 0
#         self.theta1_d = 0
#         self.theta2_d = 0
# 
#     def calculate_theta1_dd(self):
#         term1 = -self.g * (2 * self.m1 + self.m2) * np.sin(self.theta1)
#         term2 = -self.m2 * self.g * np.sin(self.theta1 - 2 * self.theta2)
#         term3 = -2 * np.sin(self.theta1 - self.theta2) * self.m2 * (
#                 self.theta2_d ** 2 * self.l2 + self.theta1_d ** 2 * self.l1 * np.cos(self.theta1 - self.theta2))
#         denominator = self.l1 * (2 * self.m1 + self.m2 - self.m2 * np.cos(2 * self.theta1 - 2 * self.theta2))
#         return (term1 + term2 + term3) / denominator
# 
#     def calculate_theta2_dd(self):
#         term1 = 2 * np.sin(self.theta1 - self.theta2) * (
#                 self.theta1_d ** 2 * self.l1 * (self.m1 + self.m2) + self.g * (self.m1 + self.m2) * np.cos(
#             self.theta1) + self.theta2_d ** 2 * self.l2 * self.m2 * np.cos(self.theta1 - self.theta2))
#         denominator = self.l2 * (2 * self.m1 + self.m2 - self.m2 * np.cos(2 * self.theta1 - 2 * self.theta2))
#         return term1 / denominator
# 
#     #chat GPTPTPTPPT
#     def step(self, dt):
#         k1_theta1_d = self.theta1_d
#         k1_theta2_d = self.theta2_d
#         k1_theta1_dd = self.calculate_theta1_dd()
#         k1_theta2_dd = self.calculate_theta2_dd()
# 
#         self.theta1 += dt / 2 * k1_theta1_d
#         self.theta2 += dt / 2 * k1_theta2_d
#         self.theta1_d += dt / 2 * k1_theta1_dd
#         self.theta2_d += dt / 2 * k1_theta2_dd
# 
#         k2_theta1_d = self.theta1_d
#         k2_theta2_d = self.theta2_d
#         k2_theta1_dd = self.calculate_theta1_dd()
#         k2_theta2_dd = self.calculate_theta2_dd()
# 
#         self.theta1 += dt / 2 * k2_theta1_d
#         self.theta2 += dt / 2 * k2_theta2_d
#         self.theta1_d += dt / 2 * k2_theta1_dd
#         self.theta2_d += dt / 2 * k2_theta2_dd
# 
#         k3_theta1_d = self.theta1_d
#         k3_theta2_d = self.theta2_d
#         k3_theta1_dd = self.calculate_theta1_dd()
#         k3_theta2_dd = self.calculate_theta2_dd()
# 
#         self.theta1 += dt * k3_theta1_d
#         self.theta2 += dt * k3_theta2_d
#         self.theta1_d += dt * k3_theta1_dd
#         self.theta2_d += dt * k3_theta2_dd
# 
#         k4_theta1_d = self.theta1_d
#         k4_theta2_d = self.theta2_d
#         k4_theta1_dd = self.calculate_theta1_dd()
#         k4_theta2_dd = self.calculate_theta2_dd()
# 
#         self.theta1 += dt / 6 * (k1_theta1_d + 2 * k2_theta1_d + 2 * k3_theta1_d + k4_theta1_d)
#         self.theta2 += dt / 6 * (k1_theta2_d + 2 * k2_theta2_d + 2 * k3_theta2_d + k4_theta2_d)
#         self.theta1_d += dt / 6 * (k1_theta1_dd + 2 * k2_theta1_dd + 2 * k3_theta1_dd + k4_theta1_dd)
#         self.theta2_d += dt / 6 * (k1_theta2_dd + 2 * k2_theta2_dd + 2 * k3_theta2_dd + k4_theta2_dd)
#         return self.theta1, self.theta2
# =============================================================================
