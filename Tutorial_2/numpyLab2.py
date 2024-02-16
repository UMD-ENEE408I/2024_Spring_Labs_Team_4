# Numpy ############################################
import numpy as np

# question 3
a = np.array([1,2,3,4])
print(a)

# question 4
b = np.ones((3,4))
print()
print(b)
c = np.zeros((4,3))
print()
print(c)

# question 5
A = np.array([[1,2,3],[4,5,6]])
B = np.array([[1,2,3,4],[5,6,7,8],[9,10,11,12]])
C = np.dot(A,B)
print(C)

# question 6
print(np.linalg.eig(np.array([[3,1],[1,2]])))