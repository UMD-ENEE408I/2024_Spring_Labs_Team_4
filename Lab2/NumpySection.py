import numpy as np

#Creates a numpy array with elements 1,2,3,4
print("Question1")
arr = np.array([1,2,3,4])
print(arr)

#creates some 2D arrays one of all zeros and one of all ones
print("Question2")
arr1 = np.ones((3,4))
arr0 = np.zeros((4,3))
print(arr1)
print(arr0)

#Performs matrix multiplication using a 3x3 and a 3x4 array
print("Question3")
A = [[1,2,3],[3,4,5]]
B = [[1,2,3,4],[5,6,7,8],[9,10,11,12]]
arr_a = np.array(A)
arr_b = np.array(B)
print(np.matmul(arr_a,arr_b))


#computes the eigen values and vectors for the matrix
print("Question4")
matrix = np.array([[3,1],[1,2]])
values,vectors = np.linalg.eig(matrix)
print("Eig Values", values)
print("Eig Vectors", vectors)

