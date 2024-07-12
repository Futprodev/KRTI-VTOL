"""
BINUS ASO KRTI VTOL DRONE 2024
Computer Vision: Teresa, Raymond
Flight Controls: Kenrich, Maul
"""

import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import cv2
from PIL import Image
import numpy as np
import RPi.GPIO as GPIO
import threading
from smbus2 import SMBus, i2c_msg

################## LIDAR CONTROL #########################
class TFminiI2C:
    def __init__(self, I2Cbus, address):
        self.I2Cbus = I2Cbus
        self.address = address

    def readDistance(self):
        write = i2c_msg.write(self.address, [1, 2, 7])
        read = i2c_msg.read(self.address, 7)
        with SMBus(self.I2Cbus) as bus:
            bus.i2c_rdwr(write, read)
            data = list(read)
            distance = data[3] << 8 | data[2]
        return distance

    def reset(self):
        reset = i2c_msg.write(self.address, [0x06])
        with SMBus(self.I2Cbus) as bus:
            bus.i2c_rdwr(reset)
            time.sleep(0.05)

# LiDAR setup
LIDAR_LEFT = TFminiI2C(1, 0x10)
LIDAR_RIGHT = TFminiI2C(1, 0x11)
LIDAR_FRONT = TFminiI2C(1, 0x12)
LIDAR_DOWN = TFminiI2C(1, 0x14)

################## Relay Control #########################
# GPIO setup
RELAY_PIN = 17  # Adjust to the correct pin number
GPIO.setmode(GPIO.BCM)
GPIO.setup(RELAY_PIN, GPIO.OUT)
GPIO.output(RELAY_PIN, GPIO.LOW)

def control_relay(state):
    if state == "ON":
        GPIO.output(RELAY_PIN, GPIO.HIGH)
        print("Relay ON, electromagnet activated")
    else:
        GPIO.output(RELAY_PIN, GPIO.LOW)
        print("Relay OFF, electromagnet deactivated")

def pick_object():
    # Descend to pick up the object
    print("Descending to pick up the object...")
    send_velocity(vehicle, 0, 0, -0.2)  # Descend at 0.2 m/s
    time.sleep(5)  # Adjust the sleep time based on required descent duration
    send_velocity(vehicle, 0, 0, 0)  # Stop descent

    # Activate the relay to pick up the object
    control_relay("ON")
    time.sleep(2)  # Wait for the electromagnet to activate
    
def drop_object():
    control_relay("OFF")
    time.sleep(2)
############################################################

################## Computer Vision #########################
# Initialize cameras
front_cam = cv2.VideoCapture(0)  # Front camera
bottom_cam = cv2.VideoCapture(1)  # Bottom camera

# Define the lower and upper bounds for orange and red for Bottom Camera
lower_bound_orange, upper_bound_orange = np.array([4, 120, 60]), np.array([23, 255, 255])
lower_bound_red1, upper_bound_red1 = np.array([0, 100, 100]), np.array([5, 255, 255])
lower_bound_red2, upper_bound_red2 = np.array([165, 100, 100]), np.array([179, 255, 255])

last_cX, last_cY = None, None

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
##############################################################

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

# Connect to the Vehicle
vehicle = connect('/dev/ttyAMA0', baud=57600, wait_ready=True)

################# Flight Control ##############################
# Function to set loiter mode (pause guided mode)
def set_loiter_mode():
    vehicle.mode = VehicleMode("LOITER")
    print("Switched to Loiter mode (pausing guided mode)")

# Function to resume guided mode
def resume_guided_mode():
    vehicle.mode = VehicleMode("GUIDED")
    print("Resumed Guided mode")

# Function to set autonomous mode
def set_autonomous_mode():
    vehicle.mode = VehicleMode("GUIDED")
    print("Switched to Autonomous control mode")

# Function to set RC mode
def set_rc_mode():
    vehicle.mode = VehicleMode("STABILIZE")
    print("Switched to RC control mode")

# Function to read terminal input
def read_terminal_input():
    global stage
    while True:
        user_input = input("Enter 'auto' for Autonomous mode, 'rc' for RC mode, 'pause' to pause, 'resume' to resume, or 'quit' to quit: ").strip().lower()
        if user_input == 'auto':
            set_autonomous_mode()
        elif user_input == 'rc':
            set_rc_mode()
        elif user_input == 'pause':
            set_loiter_mode()
        elif user_input == 'resume':
            resume_guided_mode()
        elif user_input == 'quit':
            stage = "quit"
            break
        else:
            print("Invalid input. Please enter 'auto', 'rc', 'pause', 'resume', or 'quit'.")

# Define arm and takeoff function
def arm_and_takeoff(target_altitude):
    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print("Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(target_altitude)

    while True:
        print(f"Altitude: {vehicle.location.global_relative_frame.alt}")
        if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

# Fly to a specific position
def fly_to_position(vehicle, lat, lon, alt):
    location = LocationGlobalRelative(lat, lon, alt)
    vehicle.simple_goto(location, groundspeed=5)

# Send velocity command to the vehicle
def send_velocity(vehicle, velocity_x, velocity_y, velocity_z):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_BODY_NED, 0b0000111111000111,
        0, 0, 0, velocity_x, velocity_y, velocity_z, 0, 0, 0, 0, 0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

# Drone obstacle avoidance with single turn logic
def obstacle_avoidance():
    has_turned = False  # Variable to ensure only one turn
    while True:
        front_distance = LIDAR_FRONT.readDistance()
        left_distance = LIDAR_LEFT.readDistance()
        right_distance = LIDAR_RIGHT.readDistance()

        if front_distance < 50:
            print("Obstacle detected ahead. Checking sides...")
            if left_distance > 300 and not has_turned:
                print("Turning left...")
                send_velocity(vehicle, 0, 1, 0)  # Adjust this as needed
                time.sleep(1)
                has_turned = True
            elif right_distance > 300 and not has_turned:
                print("Turning right...")
                send_velocity(vehicle, 0, -1, 0)  # Adjust this as needed
                time.sleep(1)
                has_turned = True
            else:
                print("No clear path detected. Stopping...")
                send_velocity(vehicle, 0, 0, 0)
                break
        else:
            print("No obstacle detected. Moving forward...")
            send_velocity(vehicle, 1, 0, 0)  # Move forward at 1 m/s
            time.sleep(0.1)

# Emergency signal check
def check_signal_strength(signal):
    return signal < 0.5 #Adjust strength

def read_rc_channel(channel):
    """
    Reads the specified RC channel value.
    """
    return vehicle.channels[channel]

def check_rc_switch():
    """
    Checks the RC switch to change between autonomous and RC control.
    Assumes channel 7 is used for switching.
    """
    rc_value = read_rc_channel(7)
    if rc_value > 1500:
        set_rc_mode()  # Switch to RC control
    else:
        set_autonomous_mode()  # Switch to autonomous mode
###############################################################

# Main function
def main():
    start_command_listener()
    global stage
    stage = "1"
    
    try:
        arm_and_takeoff(1)  # Take off to 1 meter altitude
        
        # Start the obstacle avoidance in a separate thread
        obstacle_avoidance_thread = threading.Thread(target=obstacle_avoidance)
        obstacle_avoidance_thread.start()

        while True:
            signal = vehicle.last_heartbeat

            # Emergency signal lost system
            if check_signal_strength(signal):
                print("Signal lost. Initiating emergency landing...")
                vehicle.mode = VehicleMode("LAND")
                break

            check_rc_switch()  # Check the RC switch position

            if stage == "1":
                # Orange object detection and pickup
                result, frame = bottom_cam.read()
                if not result:
                    break

                contours = detect_orange(frame)
                frame_height, frame_width = frame.shape[:2]
                frame_center = [frame_width // 2, frame_height // 2]

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
                    send_velocity(vehicle, velocity_x, velocity_y, 0)
                    print(f"Velocity X: {velocity_x:.2f}, Velocity Y: {velocity_y:.2f}")

                    if abs(velocity_x) < 0.1 and abs(velocity_y) < 0.1:
                        print("Orange object centered. Picking up...")
                        pick_object()
                else:
                    send_velocity(vehicle, 1, 0, 0)
                    print("Moving forward to find the orange object...")

                cv2.imshow('Bottom Camera', frame)

                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

                # Regardless of success, move to the next stage
                stage = "2"

            elif stage == "2":
                # Red object detection and drop
                result, frame = bottom_cam.read()
                if not result:
                    break

                contours = detect_red(frame)
                frame_height, frame_width = frame.shape[:2]
                frame_center = [frame_width // 2, frame_height // 2]

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
                    send_velocity(vehicle, velocity_x, velocity_y, 0)
                    print(f"Velocity X: {velocity_x:.2f}, Velocity Y: {velocity_y:.2f}")

                    if abs(velocity_x) < 0.1 and abs(velocity_y) < 0.1:
                        print("Red object centered. Dropping...")
                        drop_object()
                else:
                    send_velocity(vehicle, 1, 0, 0)
                    print("Moving forward to find the red object...")

                cv2.imshow('Bottom Camera', frame)

                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

                # Regardless of success, move to the next stage
                stage = "3"

            elif stage == "3":
                print("Mission complete. Landing...")
                vehicle.mode = VehicleMode("LAND")
                while vehicle.armed:
                    time.sleep(1)
                break

            else:
                print("Invalid stage. Please enter 1, 2, or 3.")
                break

    finally:
        print("Ensuring the drone lands and resources are cleaned up...")
        vehicle.mode = VehicleMode("LAND")
        while vehicle.armed:
            time.sleep(1)

        front_cam.release()
        bottom_cam.release()
        cv2.destroyAllWindows()
        vehicle.close()
