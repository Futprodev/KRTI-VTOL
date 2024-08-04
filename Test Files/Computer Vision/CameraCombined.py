import cv2
from PIL import Image
import numpy as np
import time

# Initialize cameras
front_cam = cv2.VideoCapture(2)  # Front camera
bottom_cam = cv2.VideoCapture(0)  # Bottom camera

# Define the lower and upper bounds for orange and red for Bottom Camera
lower_bound_orange, upper_bound_orange = np.array([4, 120, 60]), np.array([23, 255, 255])
lower_bound_red1, upper_bound_red1 = np.array([0, 100, 100]), np.array([5, 255, 255])
lower_bound_red2, upper_bound_red2 = np.array([165, 100, 100]), np.array([179, 255, 255])

# Coordinates
height1, width1, cx1, cy1 = None, None, None, None
height2, width2, cx2, cy2 = None, None, None, None

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

# Functions for Bottom Camera
def Bottom_Cam_Function(frame, contours):
    # Bottom camera
    cv2.circle(frame, (cx2, cy2), 7, (255, 0, 255), -1)
    cv2.putText(frame, f" Frame Center: ({cx2}, {cy2})", (cx2 - 70, cy2 - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest_contour)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
        cv2.drawContours(frame, [largest_contour], -1, (0, 255, 0), 2)
        cv2.circle(frame, (cX, cY), 7, (255, 0, 0), -1)
        cv2.putText(frame, f"Center: ({cX}, {cY})", (cX - 50, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    return frame, (cX, cY)

# Functions for Front Camera
def draw_polyline(frame, object_point):
    bottom_center = (width1 // 2, height1)
    points = [
        bottom_center,
        ((bottom_center[0] + object_point[0]) // 2, height1 - 50),
        ((bottom_center[0] + object_point[0]) // 2, object_point[1] + 50),
        object_point
    ]
    points = np.array(points, np.int32)
    points = points.reshape((-1, 1, 2))
    cv2.polylines(frame, [points], isClosed=False, color=(0, 255, 0), thickness=3)
    return frame

def Front_Cam_Function(frame, contours):
    cv2.line(frame, (cx1, 0), (cx1, height1), (0,0,0), 3)

    if contours:
        # Find the largest contour by area
        largest_contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest_contour)
        if M["m00"]:
        # Calculate the center of the contour
            ContourX = int(M["m10"] / M["m00"]) #center
            ContourY = int(M["m01"] / M["m00"])
            frame = draw_polyline(frame, (ContourX, ContourY))
            extLeft = tuple(largest_contour[largest_contour[:, :, 0].argmin()][0])
            extRight = tuple(largest_contour[largest_contour[:, :, 0].argmax()][0])
            cv2.circle(frame, (ContourX, ContourY), 4, (0, 255, 0), )
            cv2.circle(frame, extLeft, 2, (0, 0, 255), 3)
            cv2.circle(frame, extRight, 2, (0, 0, 255), 3)

            # Draw the largest contour
            cv2.drawContours(frame, [largest_contour], -1, (0, 255, 0), 2)
    return frame, (ContourX, ContourY)

def Detect_Gate(frame):
    #orange
    lower_limit = np.array([6, 110, 220])
    upper_limit = np.array([8, 140, 255])
    hsvimage = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsvimage, lower_limit, upper_limit)
    mask_ = Image.fromarray(mask)
    bbox = mask_.getbbox()

    if bbox is not None:
        x1, y1, x2, y2 = bbox
        frame = cv2.rectangle(frame, (x1,y1), (x2,y2), (0, 255, 0), 5)

        bcx = (x2-x1)//2 + x1
        bcy = (y2-y1)//2 + y1

        cv2.circle(frame, (bcx,bcy), 3, (255,0,0), 2) #box center

    cv2.line(frame, (cx1,0), (cx1,height1), (0,0,255), 2)
    cv2.line(frame, (0,cy1), (width1,cy1), (0,0,255), 2)
    return frame, (bcx, bcy)

# Functions for movements

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

def drone_centering(center, frame_center, pid_x, pid_y): #Bottom Cam >> diutak-atik
    x_diff = center[0] - frame_center[0]
    y_diff = center[1] - frame_center[1]
    vx = pid_x.compute(x_diff) * 0.001
    vy = pid_y.compute(y_diff) * 0.001

    max_vel = 0.5
    velocity_x = np.clip(vx, -max_vel, max_vel)
    velocity_y = np.clip(vy, -max_vel, max_vel)

    return velocity_x, velocity_y

def drone_yaw(center, frame_center, pid_yaw): #Front Cam >> diutak-atik
    x_diff = center[0] - frame_center[0]
    yaw_x = pid_yaw.compute(x_diff) * 0.001

    max_yaw = 0.5 
    yaw_x = np.clip(yaw_x, -max_yaw, max_yaw)

    return yaw_x

# Stage-based input
stage = input("Type 1/2/3 \n 1: Cari barang orange \n 2: Barang orange udh diangkat, cari ember merah & cam depan ikutin lane \n 3: cam depan cari gate \n")

# PID controllers: x and y directions for Bottom Camera & yaw for Front Camera >> diutak-atik
pid_x = PIDController(Kp=1.0, Ki=0.1, Kd=0.05, setpoint=0)
pid_y = PIDController(Kp=1.0, Ki=0.1, Kd=0.05, setpoint=0)
pid_yaw = PIDController(Kp=1.0, Ki=0.1, Kd=0.05, setpoint=0)
# Ketinggian drone pas di gate pake lidar aja

def camera_loop(cam1, cam2):
    global cx1, cy1, width1, height1, cx2, cy2, width2, height2
    global last_cX, last_cY

    res, video1 = cam1.read() #front
    res, video2 = cam2.read() #bottom

    height1, width1, _ = video1.shape
    cx1, cy1 = width1//2, height1//2

    height2, width2, _ = video2.shape
    cx2, cy2 = width2//2, height2//2

    while True:
        success1, frame1 = cam1.read()
        success2, frame2 = cam2.read()
        if not (success1 and success2):
            break

        if stage == "1":
            contours1 = detect_orange(frame1)
            frame1, object1 = Front_Cam_Function(frame1, contours1)
            contours2 = detect_orange(frame2)
            frame2, object2 = Bottom_Cam_Function(frame2, contours2)

        elif stage == "2":
            contours1 = detect_red(frame1)
            frame1, object1 = Front_Cam_Function(frame1, contours1)
            contours2 = detect_red(frame2)
            frame2, object2 = Bottom_Cam_Function(frame2, contours2)

        else:
            frame1, object1 = Detect_Gate(frame1)
            contours2 = None

        # Movements
        velocity_x, velocity_y = drone_centering(object2, (cx2, cy2), pid_x, pid_y)
        print(f"Velocity X: {velocity_x:.2f}, Velocity Y: {velocity_y:.2f}")

        yaw = drone_yaw(object1, (cx1, cy1), pid_yaw)
        print(f"Yaw: {yaw:.2f}")

        # Show
        cv2.imshow('Bottom Camera', frame2)
        cv2.imshow('Front Camera', frame1)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

        if cv2.waitKey(1) & 0xFF == ord("1"): #if pencet tombol start
            stage = "1"

        if cv2.waitKey(1) & 0xFF == ord("2"): #if object diangkat
            stage = "2"

        if cv2.waitKey(1) & 0xFF == ord("3"): #if object dilepas
            stage = "3"

    cam1.release()
    cam2.release()
    cv2.destroyAllWindows()

camera_loop(front_cam, bottom_cam)
