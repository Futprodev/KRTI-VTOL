##########################################
"""
BINUS ASO KRTI VTOL DRONE 2024
Computer Vision: Teresa, Raymond
Flight Controls: Kenrich, Maul
"""
##########################################
import time
from dronekit import connect, VehicleMode
from pymavlink import mavutil
import cv2
from PIL import Image
import numpy as np

################## Computer Vision #########################
cam = cv2.VideoCapture(0)

def Detect_Orange_Center(video): #also detects red; fix limits
    hsvimage = cv2.cvtColor(video, cv2.COLOR_BGR2HSV)
    #orange in BGR [0, 165, 255]
    #lower_limit = np.array([0, 189, 149])
    #upper_limit = np.array([16, 255, 255])
    lower_limit = np.array([6, 110, 220])
    upper_limit = np.array([8, 140, 255]) #not tested; if fail try subtracting red
    mask = cv2.inRange(hsvimage, lower_limit, upper_limit)
    
    # Replace bbox with contours
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # If any contours were found
    if contours:
        # Find the largest contour by area
        largest_contour = max(contours, key=cv2.contourArea)
        
        # Get the moments of the largest contour
        M = cv2.moments(largest_contour)
        
        if M["m00"] != 0:
            # Calculate the center of the contour
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            return (cX, cY), largest_contour
    
    return None

################## Flight Controls #########################
# Konek Ke Pihwak
connection_string = '/dev/ttyACM0'  # Sesuaiin sama address nanti
vehicle = connect(connection_string, baud=57600, wait_ready=True)
orange_detected = False

# Arming drone
def arm_and_takeoff(target_altitude):
    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(target_altitude)

    while True:
        current_altitude = vehicle.location.global_relative_frame.alt
        if current_altitude >= target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

# Adjust kecepatan motornya utk balancing
def adjust_motor_speed(roll, pitch):
    max_speed = 1000  # Maximum motor speed
    min_speed = 800   # Minimum motor speed

    # Konversi tilt dari dronenya ke motor speed
    roll_factor = 0.1  
    pitch_factor = 0.1  

    roll_speed = min(max_speed, max(min_speed, int(roll * roll_factor)))
    pitch_speed = min(max_speed, max(min_speed, int(pitch * pitch_factor)))

    # Set kecepatan motor
    vehicle.channels.overrides[mavutil.mavlink.MAV_CMD_DO_SET_ROLL_PITCH_YAW_THRUST] = (0, 0, roll_speed, pitch_speed, 0)

def orange_drone_centering(center, frame_center):
    # Calculate the difference between the center of the frame and the center of the object
    x_diff = center[0] - frame_center[0]
    y_diff = center[1] - frame_center[1]

    # Define a simple proportional control gain
    kp = 0.1
    
    # Calculate the velocity commands
    vx = -kp * y_diff  # Forward/backward adjustment
    vy = kp * x_diff   # Left/right adjustment

    # Limit the velocities to a max value
    max_vel = 0.5
    vx = np.clip(vx, -max_vel, max_vel)
    vy = np.clip(vy, -max_vel, max_vel)

    # Send velocity command to the drone
    vehicle.velocity = (vx, vy, 0)

def movement_command(velocity_x, velocity_y, velocity_z):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111, 0, 0, 0,
        velocity_x, velocity_y, velocity_z,
        0, 0, 0, 0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()

def check_signal_strength(last_heartbeat, timeout=5):
    # Check if the time since the last heartbeat exceeds the timeout
    return (time.time() - last_heartbeat) > timeout

# Terbang 5 meter
target_altitude = 5
arm_and_takeoff(target_altitude)

# Mulai timer jadi 10 detik di udara
start_time = time.time()

################## Main Loop #########################
try:
    while True:
        # if time.time() - start_time >= 20:
        #     print("20 seconds passed. Initiating landing...")
        #     vehicle.mode = VehicleMode("LAND")
        #     break
        
        signal = vehicle.last_heartbeat

        # Emergency signal lost system
        if check_signal_strength(signal):
            print("Signal lost. Initiating emergency landing...")
            vehicle.mode = VehicleMode("LAND")
            break

        result, frame = cam.read()
        if not result:
            break

        orange_center, largest_contour = Detect_Orange_Center(frame)
        video_center = (frame.shape[1] // 2, frame.shape[0] // 2)

        if orange_center:
            # Draw a circle at the center of the orange object
            cv2.circle(frame, orange_center, 10, (0, 255, 0), -1)
            # Draw lines from the center of the frame to the center of the object
            cv2.line(frame, video_center, orange_center, (255, 0, 0), 2)
            # Draw bounding box around the detected object
            x, y, w, h = cv2.boundingRect(largest_contour)
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            print("Orange object detected. Initiating landing...")
            vehicle.mode = VehicleMode("LAND")
            break
        else:
            # Move the drone forward continuously
            movement_command(1, 0, 0)  # Move x axis at 1 m/s

        cv2.imshow('Frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        time.sleep(0.1)

finally:
    # Ensure the drone lands and resources are cleaned up
    vehicle.mode = VehicleMode("LAND")
    while vehicle.armed:
        time.sleep(1)

    cam.release()
    cv2.destroyAllWindows()
    vehicle.close()