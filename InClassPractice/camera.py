import cv2

#following https://www.geeksforgeeks.org/python-opencv-capture-video-from-camera/

vid = cv2.VideoCapture(0)

while(True):
    ret,frame = vid.read()

    cv2.imshow('frame',frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

vid.release()
cv2.destroyAllWindows()