import cv2
import numpy as np
import time

#following https://www.geeksforgeeks.org/python-opencv-capture-video-from-camera/

vid = cv2.VideoCapture(0)

def findBlock():

    #timeout after 3 seconds
    timeout = time.time() + 3
    val = 0
    
    while(time.time()<timeout):
        ret,frame = vid.read()

        #convert to hsv according to the source
        into_hsv =cv2.cvtColor(frame,cv2.COLOR_BGR2HSV) 
    
        #color filters
        Blue_L_limit = np.array([100,70,70]) 
        Blue_U_limit = np.array([125,255,255]) 
        #red Wraps around so we need both
        Red_low_L_limit = np.array([0,150,150])  
        Red_low_U_limit = np.array([5,255,255])
        Red_high_L_limit = np.array([170,150,150])
        Red_high_U_limit = np.array([180,255,255])
        #green
        Green_L_limit = np.array([35,45,45]) 
        Green_U_limit = np.array([75,255,255])  
        
        #mask for the blue
        b_mask = cv2.inRange(into_hsv,Blue_L_limit,Blue_U_limit) 
        #red mask
        r_low_mask = cv2.inRange(into_hsv,Red_low_L_limit,Red_low_U_limit)
        r_high_mask = cv2.inRange(into_hsv,Red_high_L_limit,Red_high_U_limit)
        #green mask
        g_mask = cv2.inRange(into_hsv,Green_L_limit,Green_U_limit)
        
        #put in the colors
        
        blue = cv2.bitwise_and(frame,frame,mask=b_mask) 
        #combine the reds
        red_low = cv2.bitwise_and(frame,frame,mask=r_low_mask) 
        red_high = cv2.bitwise_and(frame,frame,mask=r_high_mask) 
        red = red_low + red_high

        green = cv2.bitwise_and(frame,frame,mask=g_mask) 
        
        #show the images with the masks
        
        print(frame.shape)

        #make the images smaller 
        frame = frame[140:340, 220:420]
        blue = blue[140:340, 220:420]
        red = red[140:340, 220:420]
        green = green[140:340, 220:420]
        
        
        cv2.imshow('Original',frame) 
        cv2.imshow('Blue Detector',blue) 
        cv2.imshow('Red Detector',red)
        cv2.imshow('Green Detector',green)
    
        #So now that we have the masked pictures, use a kernal to check if there is a big blob of color in the image
        #if there is then a block exists
        #count the non-zero values
        def countVals(img):
            count = 0 
            for row in range(len(img)):
                for col in range(len(img[0])):
                    if img[row][col].any() != 0:
                        count += 1
                        return count
                
        b = countVals(blue)
        r = countVals(red)
        g = countVals(green)
    
        print("blue count:",b)
        print("red count:",r)
        print("greeb count:",g)
        
        #threshold to determine if a block is here:, if there is a block, print and break
        if b > 2500:
            print("THERES A BLUE BLOCK")
            val = 1
            break;
        if r > 2500:
            print("THERES A RED BLOCK")
            val = 2
            break;
        if g > 2500:
            print("THERES A GREEN BLOCK")
            val = 3
            break;
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    vid.release()
    cv2.destroyAllWindows()
    return val


while(True):
    #look for a block
    f = findBlock()
    print("Value Returned: ", f)

    #idea -> every intersection, run the findBlock
    #if f is 0 no block is there
    #if f is 1 its blue
    #if f is 2 its red
    #if f is 3 its green
    #send this f value over wifi if its requested, and have the robot react accordingly
    #if its 0, have the robot follow the pre-determined route, if its anything else
    #have the robot enter a dodgeing sequence to get around the block depending where it is on the maze
    
    sleep(2)
