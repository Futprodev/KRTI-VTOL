import cv2
from PIL import Image
import numpy as np

cam = cv2.VideoCapture(0)

def Detect_Orange(cam):
    res, video = cam.read()
    height, width, _ = video.shape
    cx = width//2
    cy = height//2

    x1 = width//4
    x2 = width*3//4

    #orange in BGR [0, 165, 255]
    lower_limit = np.array([6, 110, 180])
    upper_limit = np.array([13, 140, 255])
    while True:
        result, video = cam.read()
        cv2.line(video, (cx, 0), (cx, height), (0,0,0), 3)
        hsvimage = cv2.cvtColor(video, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsvimage, lower_limit, upper_limit)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            # Find the largest contour by area
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            if M["m00"]:
            # Calculate the center of the contour
                ContourX = int(M["m10"] / M["m00"]) #center
                ContourY = int(M["m01"] / M["m00"])
                extLeft = tuple(largest_contour[largest_contour[:, :, 0].argmin()][0])
                extRight = tuple(largest_contour[largest_contour[:, :, 0].argmax()][0])
                #used midpoint of x due to ContourX not detecting when centered
                mid_x = (extLeft[0] + extRight[1])//2
                cv2.circle(video, (ContourX, ContourY), 4, (0, 255, 0), )
                cv2.circle(video, extLeft, 2, (0, 0, 255), 3)
                cv2.circle(video, extRight, 2, (0, 0, 255), 3)
                if ContourX < (cx-10):
                    print("turn left")
                elif ContourX > (cx+10):
                    print("turn right")
                else:
                    print("centered")
                
                #print(f"mid_x {ContourX}")
                #print(f"line_x {line_x}")

                # Draw the largest contour
                cv2.drawContours(video, [largest_contour], -1, (0, 255, 0), 2)

        cv2.imshow("Mask", mask)
        cv2.imshow("Detect Orange", video)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

def Detect_Red(cam):
    height, width, _ = video.shape
    cx = width//2
    cy = height//2

    x1 = width//4
    x2 = width*3//4
    
    #red in bgr [0, 0, 255]
    lower_limit1 = np.array([3, 190, 145]) #red and orange
    upper_limit1 = np.array([5, 245, 225]) 
    lower_limit2 = np.array([6, 110, 220]) #to remove orange
    upper_limit2 = np.array([8, 140, 255])
    while True:
        result, video = cam.read()
        cv2.line(video, (cx, 0), (cx, height), (0,0,0), 3)
        hsvimage = cv2.cvtColor(video, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsvimage, lower_limit1, upper_limit1)
        mask2 = cv2.inRange(hsvimage, lower_limit2, upper_limit2)
        mask = cv2.bitwise_and(mask1, cv2.bitwise_not(mask2))
        mask_ = Image.fromarray(mask)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            # Find the largest contour by area
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            if M["m00"]:
            # Calculate the center of the contour
                ContourX = int(M["m10"] / M["m00"]) #center
                ContourY = int(M["m01"] / M["m00"])
                extLeft = tuple(largest_contour[largest_contour[:, :, 0].argmin()][0])
                extRight = tuple(largest_contour[largest_contour[:, :, 0].argmax()][0])
                #used midpoint of x due to ContourX not detecting when centered
                mid_x = (extLeft[0] + extRight[1])//2
                cv2.circle(video, (ContourX, ContourY), 4, (0, 255, 0), )
                cv2.circle(video, extLeft, 2, (0, 0, 255), 3)
                cv2.circle(video, extRight, 2, (0, 0, 255), 3)
                if ContourX < (cx-10): #left
                    print("turn left")
                elif ContourX > (cx+10): #right
                    print("turn right")
                else:
                    print("centered")
                
                #print(f"mid_x {ContourX}")
                #print(f"line_x {line_x}")

                # Draw the largest contour
                cv2.drawContours(video, [largest_contour], -1, (0, 255, 0), 2)

        cv2.imshow("Mask", mask)
        cv2.imshow("Detect Orange", video)

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
                print("down")
            elif bcy > (cy+10):
                print("up")

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
