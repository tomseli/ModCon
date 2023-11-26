import numpy as np
import control as ct
from matplotlib import pyplot as plt

## Simulation properties ##
N = 1000
sim_time = 2
dt = sim_time/N

## System properties ##
m = 0.1
b = 0.5
k = 10
f = 1

# Matrices
A = np.array(
    [[0, 1],
    [-k/m, -b/m]]
)

B = np.array(
    [0, 1/m]
)

C = np.array(
    [1, 0]
)

D = np.array(
    [0]
)

# Initial state
x0 = np.array([0, 0])

# Output storage
x = x0
x1 = []
x2 = []
y = []

for i in range(0, N):
    x_dot = np.dot(A, x) + np.dot(B, f)
    
    x = x + x_dot * dt
    
    x1.append(x[0])
    x2.append(x[1]) 
    
    y.append(np.dot(C, x) + np.dot(D, f))


plt.figure()

plt.subplot(2, 1, 1)
plt.plot(x1, label="$x_1$") # x1 = x
plt.plot(x2, label="$x_2$") # x2 = x'
plt.title("State variables")
plt.legend()
plt.grid()

plt.subplot(2, 1, 2)
plt.plot(y, label="$y$") # y = x
plt.title("Output variables")
plt.legend()
plt.grid()

plt.tight_layout()
plt.show()
    
