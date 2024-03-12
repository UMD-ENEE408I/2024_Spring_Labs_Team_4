import cv2

#following https://www.geeksforgeeks.org/python-opencv-capture-video-from-camera/

vid = cv2.VideoCapture(0)

while(True):
    ret,frame = vid.read()
    #use haar cascade for face detection   Source: https://www.geeksforgeeks.org/face-detection-using-cascade-classifier-using-opencv-python/
    #first make it gray scale 
    frame_gray = cv2.cvtColor(frame,cv2.COLOR_RGB2GRAY)

    #downloads the classifier
    haar_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

    #scalefactor and minNeighbors are paramaters for detecting faces
    #scale factor is how large the detectionspace changes overtime
    #minneighbors will reduce false positives
    faces_rect = haar_cascade.detectMultiScale(frame_gray, scaleFactor=1.2, minNeighbors=9)

    #makes rectangles based on the ones provided by the multiscale function
    #makes them green with 0,255,0
    for (x, y, w, h) in faces_rect:
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), thickness=2) 

    cv2.imshow('Detected faces', frame)
    #cv2.imshow('frame',frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

vid.release()
cv2.destroyAllWindows()
