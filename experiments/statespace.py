import control as ct
import numpy as np
from matplotlib import pyplot as plt

N = int(1e3)
t_max = 10

m1 = 1
m2 = 1.25
M = 2
l1 = 0.4
l2 = 0.6
g = 9.81

t1 = M*l1*m1 
t2 = M*l1*m2
t3 = -M*l2*m2
t4 = l1*(m2**2)
t5 = -3*l1*(m2**2)
t6 = -l2*m1*m2
t7 = -l2*(m2**2)
num1 = t1 + t2 + t3 + t4 + t5 + t6 + t7

t1 = M*l1*l2*m1
t2 = M*l1*l2*m2
t3 = -M*(l2**2)*m2
t4 = l1*l2*(m1**2)
t5 = -3*l1*l2*(m2**2)
t6 = -(l2**2)*m1*m2
t7 = -(l2**2)*(m2**2)
num2 = t1 + t2 + t3 + t4 + t5 + t6 + t7

A = np.array(
    [[0, 0, 0, 1, 0, 0],
     [0, 0, 0, 0, 1, 0],
     [0, 0, 0, 0, 0, 1],
     [0, (g*(m1 + m2)*(-l1*m2-l2*m2))/num1, (g*(-l1*m2-l2*m2))/num1, 0, 0, 0],
     [0, (g*(m1+m2)*(M+m1-m2))/num1, (g*(M+m1-m2))/num1, 0, 0, 0],
     [0, (g*(m1+m2)*(-M*l2-l1*m2-(l2**2)*m1))/num2, (g*(-M*l2-l1*m2-l2*m1))/num2, 0, 0, 0]]
)

B = np.array(
    [[0],
     [0],
     [0],
     [(l1*m1+l1*m2-l2*m2)/num1],
     [-(2*m2)/num1],
     [(l1*m1+l1*m2-l2*m2)/num2],]
)

C = np.array(
    [[1, 0, 0, 0, 0, 0],
     [0, 1, 0, 0, 0, 0],
     [0, 0, 1, 0, 0, 0]]
)

D = np.array(
    [[0],
     [0],
     [0]]
)

R = np.array(
    [1]
)

Q = np.eye(6)

# Q = np.array(
#     [[1, 0, 0, 0],
#      [0, 1, 0, 0],
#      [0, 0, 1, 0],
#      [0, 0, 0, 1]]
# )

time = np.arange(0, t_max, step=(t_max/float(N)))

K, S, E = ct.lqr(A, B, Q, R)

sys = ct.ss(A-B@K, B, C, D)
# sys = ct.ss(A, B, C, D)
t, y = ct.impulse_response(sys, time)

fig, (ax1, ax2) = plt.subplots(1, 2)
ax1.plot(t, y[0][0], label="$x$")
ax1.plot(t, y[1][0], label="$\\theta_1$")
ax1.plot(t, y[2][0], label="$\\theta_2$")
ax1.legend()
ax1.grid()

# x = [element.real for element in E]
# y = [element.imag for element in E]

# ax2.scatter(x, y)
# ax2.set_ylabel("Imag")
# ax2.set_xlabel("Real")
# ax2.set_xlim(np.amin(x)-5, 10)
# ax2.grid()
plt.show()
