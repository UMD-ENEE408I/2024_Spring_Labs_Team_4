
import numpy as np

from numpy import linalg as LA
a = np.array([1,2,3,4])

b = np.ones((3,4))

c = np.zeros((4,3))

d = np.array([[1.5, 2, 3], [4, 5, 6]])

e = np.array([[1, 2, 3, 4], [4, 5, 6, 7],[1, 2, 3, 1]])


mult = np.matmul(d,e)

print(mult)
f = np.array([[3,1],[1,2]])
eigenvalues, eigenvectors = LA.eig(f)

print(eigenvalues, eigenvectors)