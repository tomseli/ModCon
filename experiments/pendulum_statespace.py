import numpy as np
from matplotlib import pyplot as plt
from scipy.linalg import solve_continuous_are

## Simulation properties ##
N = 1000
T = 10
dt = T/N

## System properties ##
g = 9.81
I = 0.006
m = 0.2
M = 0.5
l = 0.3
b = 0.1

p = I*(M+m) + M*m*l**2

# Matrices
# Ingevuld naar:
# https://ctms.engin.umich.edu/CTMS/index.php?example=InvertedPendulum&section=SystemModeling
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

Q = np.eye(4)
R = np.eye(1)

# Control
P = solve_continuous_are(A, B, Q, R)
K = np.dot(np.linalg.inv(R), np.dot(B.T, P))

# Initial state
x0 = np.array(
    [[0.1],
     [0.0],
     [0.0],
     [0.0]]
)
x = x0

# Output storage
t = []
x1 = []
x2 = []
x3 = []
x4 = []
y1 = []
y2 = []
u_out = []

def u(t, step):
    return np.array([[1.0]]) if t > step else np.array([[0.0]])

for i in range(0, N):
    if(i == 0):
        t.append(0)
    else:
        t.append(t[-1] + dt)

    u_lqr = -np.dot(K, x)
    u_out.append(u_lqr)

    x_dot = A@x + B@u_lqr
    x = x + x_dot * dt

    y_out = C@x + D@u_lqr
    y1.append(y_out[0])
    y2.append(y_out[1])

    x1.append(x[0])
    x2.append(x[1])
    x3.append(x[2])
    x4.append(x[3])

## Plots ##
plt.figure(figsize=(6, 10))

plt.subplot(3, 1, 1)
plt.plot(t, x1, label="$x_1$") # x1 = x
plt.plot(t, x2, label="$x_2$") # x2 = x'
plt.plot(t, x3, label="$x_3$") # x3 = theta
plt.plot(t, x4, label="$x_4$") # x4 = theta'
plt.title("State variables")
plt.legend()
plt.grid()

plt.subplot(3, 1, 2)
plt.plot(t, y1, label="$y_1$") # y = x
plt.plot(t, y2, label="$y_2$") # y = theta
plt.title("Output variables")
plt.legend()
plt.grid()

# plt.subplot(3, 1, 3)
# plt.plot(u, label="$u$") # u = f
# plt.title("Control")
# plt.legend()
# plt.grid()

plt.show()