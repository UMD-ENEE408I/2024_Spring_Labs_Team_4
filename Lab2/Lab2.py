#This is the origional file worked in, this code is identical to the other python files however
#the other files are broken down into the different sections of this assignment

import numpy as np
import matplotlib.pyplot as plt
import math
import scipy
from scipy.optimize import fsolve,minimize
from scipy.fft import fft,fftfreq
import cv2


print("Question1")
arr = np.array([1,2,3,4])
print(arr)

print("Question2")
arr1 = np.ones((3,4))
arr0 = np.zeros((4,3))
print(arr1)
print(arr0)

print("Question3")
A = [[1,2,3],[3,4,5]]
B = [[1,2,3,4],[5,6,7,8],[9,10,11,12]]
arr_a = np.array(A)
arr_b = np.array(B)
print(np.matmul(arr_a,arr_b))

print("Question4")
matrix = np.array([[3,1],[1,2]])
values,vectors = np.linalg.eig(matrix)
print("Eig Values", values)
print("Eig Vectors", vectors)


#MATPLOTLIB Questions 1,2
print("MATPLOTLIB QUESTIONS")
x = np.linspace(0,2*math.pi)
y = np.sin(x)

plt.plot(x,y)
plt.xlabel("x Axis")
plt.ylabel('y axis')
plt.show()

#MATPLOT LIB Q 3
x = np.linspace(0,2*math.pi)
y = np.linspace(0,2*math.pi)
x,y = np.meshgrid(x,y)
z = np.sin(np.sqrt(x**2 + y**2))
fig,ax = plt.subplots(subplot_kw={"projection": "3d"})

ax.plot_surface(x,y,z)

plt.show()


#SCIPY
print("SCIPY Question 1")
def eq(x):
    return [3*x[0] + x[1] - 9,x[0] + 2*x[1] - 8]
print(fsolve(eq,[0,0]))
print("SCIPY Question 2")
def eq2(x):
    return [(x[0]**2) + (2*x[0])]
print(minimize(eq2,[-1,-1])
      )


N = 500
T = 1/1000


#follows this : https://docs.scipy.org/doc/scipy/tutorial/fft.html
x = np.linspace(0,N*T,N)
y = np.sin(x*100*math.pi) + 0.5*np.sin(160*math.pi*x)
yf = fft(y)
xf = fftfreq(N, T)[:N//2]
plt.plot(xf, 2.0/N * np.abs(yf[0:N//2]))
plt.grid()
plt.show()

#OPEN CV
#image used: https://images.pexels.com/photos/209296/pexels-photo-209296.jpeg?cs=srgb&dl=architecture-building-buy-209296.jpg&fm=jpg
img = cv2.imread('house_pic.jpg')
cv2.imshow('image',img)

img_gray = cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)
cv2.imshow('image2',img_gray)

#use cv.canny
img_edge = cv2.Canny(img_gray,100,200)
cv2.imshow('image3', img_edge)

#use haar cascade for face detection   Source: https://www.geeksforgeeks.org/face-detection-using-cascade-classifier-using-opencv-python/
img_friends = cv2.imread('friends3.JPG')
#first make it gray scale 
img_friends_gray = cv2.cvtColor(img_friends,cv2.COLOR_RGB2GRAY)

#downloads the classifier
haar_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml') 

#scalefactor and minNeighbors are paramaters for detecting faces
#scale factor is how large the detectionspace changes overtime
#minneighbors will reduce false positives
faces_rect = haar_cascade.detectMultiScale(img_friends_gray, scaleFactor=1.05, minNeighbors=9) 

#makes rectangles based on the ones provided by the multiscale function
#makes them green with 0,255,0
for (x, y, w, h) in faces_rect: 
	cv2.rectangle(img_friends, (x, y), (x+w, y+h), (0, 255, 0), thickness=2) 

cv2.imshow('Detected faces', img_friends) 



cv2.waitKey(0)  #nessassary for some reason: https://www.geeksforgeeks.org/python-opencv-cv2-imshow-method/
cv2.destroyAllWindows() 