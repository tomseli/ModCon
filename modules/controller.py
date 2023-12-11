from matplotlib import pyplot as plt
import numpy as np
from scipy.linalg import solve_continuous_are
from modules import lowpass
from modules import physics


class ControlPID():
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

        self.lp_filter = None
        self.limit = 0

        self.logging = False
  
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

        if self.lp_filter is not None:
            self.sum = self.lp_filter.filter(self.sum, dt)
        
        if(self.limit != 0):
            self.sum = np.clip(self.sum, -self.limit, self.limit)

        if(self.logging):
            if(len(self.t_log) != 0):
                self.t_log.append(self.t_log[-1] + dt)
            else:
                self.t_log.append(0)
            self.sp_log.append(setpoint)
            self.a_log.append(current)
            self.e_log.append(self.e)
            self.p_log.append(self.p_out)
            self.i_log.append(self.i_out)
            self.d_log.append(self.d_out)
            self.sum_log.append(self.sum)

        return self.sum
    
    def addLPFilter(self, tau : float) -> None:
        self.lp_filter = lowpass.LowPass(tau)
        return None

    def addLimit(self, limit: float) -> None:
        self.limit = limit
        return None
    
    def print(self):
        print("P {:.4f} \t I {:.4f} \t D {:.4f} \t SUM {:.4f} \t E {:.4f}".format(
            self.p_out, self.i_out, self.d_out, self.sum, self.e
        ))

    def enableLogging(self):
        self.logging = True

        self.t_log = []
        self.a_log = []
        self.sp_log = []
        self.e_log = []
        self.p_log = []
        self.i_log = []
        self.d_log = []
        self.sum_log = []

    def plotLogs(self, title : str, block : bool) -> None:
        fig, axs = plt.subplots(2, 2)
        fig.suptitle(title)

        # Error
        axs[0, 0].plot(self.t_log, self.e_log)
        axs[0, 0].set_title("Error")
        axs[0, 0].grid()

        # Error with SP
        axs[0, 1].plot(self.t_log, [sum(x) for x in zip(self.e_log, self.sp_log)], label="Error")
        axs[0, 1].plot(self.t_log, self.sp_log, label="Setpoint")
        axs[0, 1].plot(self.t_log, self.a_log, label="Actual")
        axs[0, 1].set_title("Superposition error \n with sp")
        axs[0, 1].legend()
        axs[0, 1].grid()

        # PID values 
        axs[1, 0].plot(self.t_log, self.p_log, label="P")
        axs[1, 0].plot(self.t_log, self.i_log, label="I")
        axs[1, 0].plot(self.t_log, self.d_log, label="D")
        axs[1, 0].set_title("PID Values")
        axs[1, 0].legend()
        axs[1, 0].grid()

        # PID Output
        axs[1, 1].plot(self.t_log, self.sum_log)
        axs[1, 1].set_title("PID Output")
        axs[1, 1].grid()
         
        plt.show(block=block)
        
class ControlLQR():
    def __init__(self, pendulum = physics.PhysicsDoublePendulum) -> None:
        self.M = 0
        self.m1 = 0
        self.m2 = 0
        self.l1 = 0
        self.l2 = 0
        self.g = 9.81

        self.pendulum = pendulum
        return

    def init(self) -> None:
        m1 = self.m1
        m2 = self.m2
        M = self.M
        l1 = self.l1
        l2 = self.l2
        g = self.g

        num1 = \
        + M*l1*m1 
        + M*l2*m2 
        - M*l2*m2 
        + l1*(m2**2) 
        - 3*l1*(m2**2) 
        - l2*m1*m2 
        -l2*(m2**2) 

        num2 = \
        + M*l1*l2*m1
        + M*l1*l2*m2
        - M*(l2**2)*m2
        + l1*l2*(m1**2)
        - 3*l1*l2*(m2**2)
        - (l2**2)*m1*m2
        - (l2**2)*(m2**2)

        self.A = np.array(
            [[0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1],
            [0, -g*(m1 + m2)/M, 0, 0, 0, 0],
            [0, g*(M + m1)*(m1 + m2)/(M*l1*m1), -g*m2/(l1*m1), 0, 0, 0],
            [0, -g*(m1 + m2)/(l2*m1), g*(m1 + m2)/(l2*m1), 0, 0, 0]]
        )

        self.B = np.array(
            [[0],
            [0],
            [0],
            [1/M],
            [-1/M*l1],
            [0]]
        )


        self.R = np.array(
            [[0.2]]
        )
        
        i = 1
        self.Q = np.array(
            [[i+30, 0, 0, 0, 0, 0],
             [0, i+10, 0, 0, 0, 0],
             [0, 0, i, 0, 0, 0],
             [0, 0, 0, i+30, 0, 0],
             [0, 0, 0, 0, i+10, 0],
             [0, 0, 0, 0, 0, i]]
        )

        self.P = solve_continuous_are(self.A, self.B, self.Q, self.R)
        self.K = np.linalg.inv(self.R)@(self.B.T@self.P)
        return
    
    def control(self) -> float:
        x1 = self.pendulum.x
        x2 = self.pendulum.theta1
        x3 = self.pendulum.theta2
        x4 = self.pendulum.xc_d
        x5 = self.pendulum.theta1_d
        x6 = self.pendulum.theta2_d

        X = np.array(
            [[x1],
             [x2],
             [x3],
             [x4],
             [x5],
             [x6]]
        )

        return np.dot(-self.K, X)[0][0]