import sympy as sp

# Define symbolic variables
M, m1, m2, l1, l2, g = sp.symbols('M m1 m2 l1 l2 g')

# Define matrices A and B
A = sp.Matrix([[1, 0, 0, 0, 0, 0],
               [0, 1, 0, 0, 0, 0],
               [0, 0, 1, 0, 0, 0],
               [0, 0, 0, (M+m1+m2), ((0.5 *m1)+m2)*l1, (1/2)*m2*l2],
               [0, 0, 0, (((1/2)*m1)+m2)*l1, (((1/3)*m1)+m2)*(l1**2), (1/2)*m2*l1*l2],
               [0, 0, 0, (1/2)*m2*l2, (1/2)*m2*l1*l2, (1/3)*m2*(l2**2)]])

N = sp.Matrix([[0, 0, 0, 1, 0, 0],
               [0, 0, 0, 0, 1, 0],
               [0, 0, 0, 0, 0, 1],
               [0, 0, 0, 0, 0, 0],
               [0, -((l1 * g)/2)*(m1+(2*m2)), 0, 0, 0, 0],
               [0, 0, -((l2 * g)/2)*m2, 0, 0, 0]])

F = sp.Matrix([[0],
               [0],
               [0],
               [1],
               [0],
               [0]])

# Calculate the product of the inverse of A and B symbolically
C = A.inv() *N

# Print the result
print("Resultant Matrix C:")
print(C)
