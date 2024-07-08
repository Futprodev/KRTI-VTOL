import cv2
from PIL import Image
import numpy as np
import time
import threading

# Initialize cameras
front_cam = cv2.VideoCapture(0)  # Front camera
bottom_cam = cv2.VideoCapture(0)  # Bottom camera

# Define the lower and upper bounds for orange and red for Bottom Camera
lower_bound_orange, upper_bound_orange = np.array([4, 120, 60]), np.array([23, 255, 255])
lower_bound_red1, upper_bound_red1 = np.array([0, 100, 100]), np.array([5, 255, 255])
lower_bound_red2, upper_bound_red2 = np.array([165, 100, 100]), np.array([179, 255, 255])

# Coordinates for Bottom Camera
last_cX, last_cY = None, None

# Coordinates for Front Camera
height, width, cx, cy = None, None, None, None

def detect_orange(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask_orange = cv2.inRange(hsv, lower_bound_orange, upper_bound_orange)
    contours_orange, _ = cv2.findContours(mask_orange, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return contours_orange

def detect_red(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask_red1 = cv2.inRange(hsv, lower_bound_red1, upper_bound_red1)
    mask_red2 = cv2.inRange(hsv, lower_bound_red2, upper_bound_red2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)
    contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return contours_red

def drone_centering(center, frame_center, pid_x, pid_y):
    x_diff = center[0] - frame_center[0]
    y_diff = center[1] - frame_center[1]
    vx = pid_x.compute(x_diff) * 0.001
    vy = pid_y.compute(y_diff) * 0.001

    max_vel = 0.5
    velocity_x = np.clip(vx, -max_vel, max_vel)
    velocity_y = np.clip(vy, -max_vel, max_vel)

    return velocity_x, velocity_y

class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint=0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.last_error = 0
        self.integral = 0
        self.last_time = time.time()

    def compute(self, measured_value):
        current_time = time.time()
        delta_time = current_time - self.last_time
        error = self.setpoint - measured_value
        P_out = self.Kp * error
        self.integral += error * delta_time
        I_out = self.Ki * self.integral
        delta_error = error - self.last_error
        D_out = self.Kd * (delta_error / delta_time) if delta_time > 0 else 0
        output = P_out + I_out + D_out
        self.last_error = error
        self.last_time = current_time
        return output

# PID controllers for x and y directions for Bottom Camera
pid_x = PIDController(Kp=1.0, Ki=0.1, Kd=0.05, setpoint=0)
pid_y = PIDController(Kp=1.0, Ki=0.1, Kd=0.05, setpoint=0)

# Functions for Front Camera
def Detect_Orange(cam):
    result, video = cam.read()
    contours = detect_orange(video)
    cv2.line(video, (cx, 0), (cx, height), (0,0,0), 3)

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

    cv2.imshow("Detect Orange", video)

def Detect_Red(cam):
    result, video = cam.read()
    contours = detect_red(video)
    cv2.line(video, (cx, 0), (cx, height), (0,0,0), 3)

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

    cv2.imshow("Detect Red", video)

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

# Stage-based input
stage = input("Type 1/2/3 \n 1: Cari barang orange \n 2: Barang orange udh diangkat, cari ember merah & cam depan ikutin lane \n 3: cam depan cari gate \n")

# kamera depan 
def front_camera_loop(cam):
    global cx, cy
    res, video = cam.read()
    height, width, _ = video.shape
    cx = width//2
    cy = height//2

    x1 = width//4
    x2 = width*3//4
    while True:
        success, frame = cam.read()
        if not success:
            break

        if stage == "1":
            Detect_Orange(cam)
        elif stage == "2":
            Detect_Red(cam)
        else:
            Detect_Gate(cam)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
    cam.release()
    cv2.destroyAllWindows()

# kamera bawah 
def bottom_camera_loop(cam):
    global last_cX, last_cY
    while True:
        success, frame = cam.read()
        if not success:
            break

        if stage == "1":
            contours = detect_orange(frame)
        elif stage == "2":
            contours = detect_red(frame)
        else:
            contours = None

        frame_height, frame_width = frame.shape[:2]
        frame_center = [frame_width // 2, frame_height // 2]
        cv2.circle(frame, (frame_center[0], frame_center[1]), 7, (255, 0, 255), -1)
        cv2.putText(frame, f" Frame Center: ({frame_center[0]}, {frame_center[1]})", (frame_center[0] - 70, frame_center[1] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                last_cX, last_cY = cX, cY
            cv2.drawContours(frame, [largest_contour], -1, (0, 255, 0), 2)

        if last_cX is not None and last_cY is not None:
            cv2.circle(frame, (last_cX, last_cY), 7, (255, 0, 0), -1)
            cv2.putText(frame, f"Center: ({last_cX}, {last_cY})", (last_cX - 50, last_cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            center = [last_cX, last_cY]

            velocity_x, velocity_y = drone_centering(center, frame_center, pid_x, pid_y)
            print(f"Velocity X: {velocity_x:.2f}, Velocity Y: {velocity_y:.2f}")

        cv2.imshow('Bottom Camera', frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cam.release()
    cv2.destroyAllWindows()

# Start detekssi loop utk tiap kamera
front_thread = threading.Thread(target=front_camera_loop, args=(front_cam,))
bottom_thread = threading.Thread(target=bottom_camera_loop, args=(bottom_cam,))

front_thread.start()
bottom_thread.start()

front_thread.join()
bottom_thread.join()
