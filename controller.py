from matplotlib import pyplot as plt
import numpy as np
import lowpass

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
        

