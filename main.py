from matplotlib import pyplot as plt
import numpy as np

N = int(10e3)
t = 3
dt = t/N

g = 9.81
l = 0.25

time = np.zeros(N)

theta = np.zeros(N)
theta_d = np.zeros(N)
theta_dd = np.zeros(N)

theta[0] = 2

for i in range(1, N):
    time[i] = time[i-1] + dt

    theta_dd[i] = -g/l * np.sin(theta[i-1])
    theta_d[i] = theta_d[i-1] + theta_dd[i] * dt
    theta[i] = theta[i-1] + theta_d[i] * dt

plt.plot(time, theta)
plt.show()


