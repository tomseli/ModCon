import numpy as np
from matplotlib import pyplot as plt
from scipy.linalg import solve_continuous_are

## Simulation properties ##
N = 1000
T = 5
dt = T/N

## System properties ##
m = 0.2
b = 0.5
k = 10 

# Matrices
A = np.array(
    [[0.0, 1.0],
    [-k/m, -b/m]]
)

B = np.array(
    [[0.0], 
     [1/m]]
)

C = np.array(
    [1.0, 0.0]
)

D = np.array(
    [0.0]
)

# Q = np.array([
#     [1.0, 0.0],
#     [0.0, 5.5]])

Q = np.eye(2)
R = np.eye(1)

# TODO: Uitzoeken wat hier gaande is
P = solve_continuous_are(A, B, Q, R)
K = np.dot(np.linalg.inv(R), np.dot(B.T, P))

# Initial state
x0 = np.array([[10.0], [0.0]])
x = x0

# Output storage for plotting
x1 = []
x2 = []
y = []
u = []

## Simulation ##
for i in range(0, N):
    u_lqr = -np.dot(K, x)
    u.append(u_lqr[0])

    x_dot = A@x + B@u_lqr
    
    x = x + x_dot * dt

    x1.append(x[0])
    x2.append(x[1]) 
    
    y_out = C@x + D@u_lqr
    y.append(y_out)

## Plots ##
plt.figure(figsize=(6, 10))

plt.subplot(3, 1, 1)
plt.plot(x1, label="$x_1$") # x1 = x
plt.plot(x2, label="$x_2$") # x2 = x'
plt.title("State variables")
plt.legend()
plt.grid()

plt.subplot(3, 1, 2)
plt.plot(y, label="$y$") # y = x
plt.title("Output variables")
plt.legend()
plt.grid()

plt.subplot(3, 1, 3)
plt.plot(u, label="$u$") # u = f
plt.title("Control")
plt.legend()
plt.grid()

plt.tight_layout()
plt.show()
