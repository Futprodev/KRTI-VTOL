import cv2
from PIL import Image
import numpy as np

cam = cv2.VideoCapture(0)

def Detect_Orange(cam):
    #orange in BGR [0, 165, 255]
    lower_limit = np.array([6, 110, 220])
    upper_limit = np.array([8, 140, 255])
    while True:
        result, video = cam.read()
        hsvimage = cv2.cvtColor(video, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsvimage, lower_limit, upper_limit)
        mask_ = Image.fromarray(mask)
        bbox = mask_.getbbox()

        height, width, z = video.shape
        cx = width//2
        cy = height//2

        if bbox is not None:
            x1, y1, x2, y2 = bbox
            frame = cv2.rectangle(video, (x1,y1), (x2,y2), (0, 255, 0), 5)

            bcx = (x2-x1)//2 + x1
            bcy = (y2-y1)//2 + y1

            cv2.circle(video, (bcx,bcy), 3, (255,0,0), 2) #box center

            if bcx < (cx-20):
                print("yaw right")
            elif bcx > (cx+20):
                print("yaw left")

        cv2.line(video, (cx,0), (cx,height), (0,0,255), 2)
        cv2.line(video, (0,cy), (width,cy), (0,0,255), 2)

        #cv2.imshow("HSV", hsvimage)
        cv2.imshow("Mask", mask)
        cv2.imshow("Detect Orange", video)


        if cv2.waitKey(1) & 0xFF == ord('q'): #find a way to end after orange detected?
            break

    cv2.destroyAllWindows()

def Detect_Red(cam): #detects orange as well
    #red in bgr [0, 0, 255]
    lower_limit1 = np.array([3, 190, 145]) #red and orange
    upper_limit1 = np.array([5, 245, 225]) 
    lower_limit2 = np.array([6, 110, 220]) #to remove orange
    upper_limit2 = np.array([8, 140, 255])
    while True:
        result, video = cam.read()
        hsvimage = cv2.cvtColor(video, cv2.COLOR_BGR2HSV)

        mask1 = cv2.inRange(hsvimage, lower_limit1, upper_limit1)
        mask2 = cv2.inRange(hsvimage, lower_limit2, upper_limit2)
        #mask = mask1 - mask2
        mask = cv2.bitwise_and(mask1, cv2.bitwise_not(mask2)) #not tested
        mask_ = Image.fromarray(mask)
        bbox = mask_.getbbox()

        height, width, z = video.shape
        cx = width//2
        cy = height//2

        if bbox is not None:
            x1, y1, x2, y2 = bbox
            frame = cv2.rectangle(video, (x1,y1), (x2,y2), (0, 255, 0), 5)

            bcx = (x2-x1)//2 + x1
            bcy = (y2-y1)//2 + y1

            cv2.circle(video, (bcx,bcy), 3, (255,0,0), 2) #box center

            if bcx < (cx-20):
                print("yaw right")
            elif bcx > (cx+20):
                print("yaw left")
        

        cv2.line(video, (cx,0), (cx,height), (0,0,255), 2)
        cv2.line(video, (0,cy), (width,cy), (0,0,255), 2)

        #cv2.imshow("HSV", hsvimage)
        cv2.imshow("Mask", mask)
        cv2.imshow("Detect Red", video)


        if cv2.waitKey(1) & 0xFF == ord('q'): 
            break
    
    cv2.destroyAllWindows()

def Detect_Gate(cam):
    #orange
    lower_limit = np.array([6, 110, 220])
    upper_limit = np.array([8, 140, 255])
    while True:
        result, video = cam.read()
        hsvimage = cv2.cvtColor(video, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsvimage, lower_limit, upper_limit)
        mask_ = Image.fromarray(mask)
        bbox = mask_.getbbox()

        height, width, z = video.shape
        cx = width//2
        cy = height//2

        if bbox is not None:
            x1, y1, x2, y2 = bbox
            frame = cv2.rectangle(video, (x1,y1), (x2,y2), (0, 255, 0), 5)

            bcx = (x2-x1)//2 + x1
            bcy = (y2-y1)//2 + y1

            cv2.circle(video, (bcx,bcy), 3, (255,0,0), 2) #box center

            if bcx < (cx-20):
                print("yaw right")
            elif bcx > (cx+20):
                print("yaw left")

            if bcy < (cy-10):
                print("up")
            elif bcy > (cy+10):
                print("down")

        cv2.line(video, (cx,0), (cx,height), (0,0,255), 2)
        cv2.line(video, (0,cy), (width,cy), (0,0,255), 2)

        #cv2.imshow("HSV", hsvimage)
        cv2.imshow("Mask", mask)
        cv2.imshow("Detect Gate", video)


        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
    
        
#detect orange (object) then detect red (bucket) then gate (orange)
Detect_Orange(cam)
Detect_Red(cam)
Detect_Gate(cam)
cam.release()
