import control as ct
import numpy as np
from matplotlib import pyplot as plt

m = 0.2
M = 0.5
b = 0.1
I = 0.006
g = 9.81
l = 0.3

N = int(1e3)
t_max = 10

p = I*(m + M)+M*m*l**2

A = np.array(
    [[0, 1, 0, 0], 
    [0, -(I+m*l**2)*b/p, (m**2*g*l**2)/p, 0], 
    [0, 0, 0, 1], 
    [0, -m*l*b/p, m*g*l*(M+m)/p, 0]]
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

R = np.array(
    [0.1]
)

# Q = np.array(
#     [[1, 0, 0, 0],
#      [0, 1, 0, 0],
#      [0, 0, 1, 0],
#      [0, 0, 0, 1]]
# )

Q = np.array(
    [[1, 0, 0, 0],
     [0, 1, 0, 0],
     [0, 0, 1, 0],
     [0, 0, 0, 1]]
)

time = np.arange(0, t_max, step=(t_max/float(N)))

K, S, E = ct.lqr(A, B, Q, R)

sys = ct.ss(A-B@K, B, C, D)
y, t = ct.impulse_response(sys, time)

fig, (ax1, ax2) = plt.subplots(1, 2)
ax1.plot(t, y[0][0], label="$x$")
ax1.plot(t, y[1][0], label="$\\theta$")
ax1.legend()
ax1.grid()

x = [element.real for element in E]
y = [element.imag for element in E]

ax2.scatter(x, y)
ax2.set_ylabel("Imag")
ax2.set_xlabel("Real")
ax2.set_xlim(np.amin(x)-5, 10)
ax2.grid()
plt.show()
