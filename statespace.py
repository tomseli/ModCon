from scipy import signal, linalg
import numpy as np
from matplotlib import pyplot as plt

m = 1
M = 1
b = 1
I = 1
g = 9.81
l = 1

N = int(1e6)
t_max = 2

p = I*(m + M)+M*m*l**2

A = np.array(
    [[0, 1, 0, 0], 
    [0, -(I+m*l**2)*b/p, (m**2*g*l**2)/p, 0], 
    [0, 0, 0, 1], 
    [0, -(m*l*b)/p, m*g*l*(M+m)/p, 0]]
)

B = np.array(
    [[0],
    [(I+m*l**2)/p],
    [0],
    [m*l/p]]
)

C = np.array(
    [[1, 0, 0, 0],
    [0, 0, 1, 0]]
)

D = np.array(
    [[0],
    [0]]
)

time = np.arange(0, int(t_max), step=(t_max/float(N)))

system = signal.StateSpace(A, B, C, D)
t, y = signal.step(system, T=time)

plt.plot(t, y)
plt.grid()
plt.show()
