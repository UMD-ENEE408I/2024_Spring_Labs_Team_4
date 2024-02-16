import cv2 as cv
from matplotlib import pyplot as plt

img = cv.imread("house.png")
img2 = cv.resize(img, (960, 540))
cv.imshow("Original", img2)
cv.waitKey(0)

gray_image = cv.cvtColor(img2, cv.COLOR_BGR2GRAY)

cv.imshow('Grayscale', gray_image)
cv.waitKey(0)

edges = cv.Canny(img,100,200)
plt.subplot(121),plt.imshow(img,cmap = 'gray')
plt.title('Original Image'), plt.xticks([]), plt.yticks([])
plt.subplot(122),plt.imshow(edges,cmap = 'gray')
plt.title('Edge Image'), plt.xticks([]), plt.yticks([])
plt.show() 

cv.waitKey(0)
  
# Reading the image 
img = cv.imread('face_recog_image.jpg') 
  
# Converting image to grayscale 
gray_img = cv.cvtColor(img, cv.COLOR_BGR2GRAY) 
  
# Loading the required haar-cascade xml classifier file 
haar_cascade = cv.CascadeClassifier('Haarcascade_frontalface_default.xml') 
  
# Applying the face detection method on the grayscale image 
faces_rect = haar_cascade.detectMultiScale(gray_img, 1.1, 9) 
  
# Iterating through rectangles of detected faces 
for (x, y, w, h) in faces_rect: 
    cv.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2) 
  
cv.imshow('Detected faces', img) 
cv.waitKey(0)