import sympy as sp

# Define symbolic variables
M, m1, m2, l1, l2, g = sp.symbols('M m1 m2 l1 l2 g')

# Define matrices A and B
A = sp.Matrix([[1, 0, 0, 0, 0, 0],
               [0, 1, 0, 0, 0, 0],
               [0, 0, 1, 0, 0, 0],
               [0, 0, 0, (M+m1), m2*l1, -m2*l2],
               [0, 0, 0, -1, l2, l2],
               [0, 0, 0, m2, (m1+m2)*l1, m2*l2]])

N = sp.Matrix([[0, 0, 0, 1, 0, 0],
               [0, 0, 0, 0, 1, 0],
               [0, 0, 0, 0, 0, 1],
               [0, 0, 0, 0, 0, 0],
               [0, 0, 0, 0, 0, 0],
               [0, (m1+m2)*g, g, 0, 0, 0]])

F = sp.Matrix([[0],
               [0],
               [0],
               [1],
               [0],
               [0]])

# Calculate the product of the inverse of A and B symbolically
C = A.inv() * F

# Print the result
print("Resultant Matrix C:")
print(C)
