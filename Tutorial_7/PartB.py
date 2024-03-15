import cv2
import robotpy_apriltag
import numpy as np

#AprilTag python resource found on github: https://github.com/robotpy/robotpy-apriltag/blob/main/tests/test_detection.py
#

#start with opening a camera on the laptop
#open a camera
vid = cv2.VideoCapture(0)
#initalize the detector from the library
detector = robotpy_apriltag.AprilTagDetector()
detector.addFamily("tag36h11")
while(True):
    #get the current frame from the camera
    ret,frame = vid.read()

    #gray scale the image
    frame_gray = cv2.cvtColor(frame,cv2.COLOR_RGB2GRAY)

    #run detection
    res = detector.detect(frame_gray)


    #plot the box on the image

    #get the corner points
    if res != []:

        #gets opposite corners of the apriltag
        point1 = res[0].getCorner(0)
        point2 = res[0].getCorner(2)
        #get the infromation from the points
        #smallest x and y point and top x and y point
    
        #plot the rectangle
        cv2.rectangle(frame, (int(point1.x),int(point1.y)), (int(point2.x),int(point2.y)), (0, 255, 0), thickness=2) 
       

    cv2.imshow('Camera', frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

vid.release()
cv2.destroyAllWindows()


#QUESTION 2


#The axies show that 
#zA == Zb
#xa rotated 90 about z is xb 
#ya rotated 90 about z is yb

#so we want to use the Rz 3D rotation matrix

#Rz = cos(x) -sin(x) 0
#     sin(x)  cos(x) 0
#      0       0     1

#so for 90 degrees

Rz = [[0,-1,0],[1,0,0],[0,0,1]]
print(Rz)


#Question 3

#B = t + A

t = [1,2,3]

#so if Pb = [3,4,5]
#first solve Ax = pb for A the rotation matrix
#then subtract t

A = np.array(Rz)
b = np.array([3,4,5])

x = np.linalg.solve(A,b)

ans = x - t

print("solution: ", ans)