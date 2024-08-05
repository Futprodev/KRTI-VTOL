import time
from dronekit import connect, VehicleMode
from pymavlink import mavutil
from smbus2 import SMBus, i2c_msg
import threading
import cv2

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
        with SMBus(self.I2bus) as bus:
            bus.i2c_rdwr(reset)
            time.sleep(0.05)

# LiDAR setup
LIDAR_LEFT = TFminiI2C(1, 0x30)
LIDAR_RIGHT = TFminiI2C(1, 0x11)
LIDAR_FRONT = TFminiI2C(1, 0x12)
LIDAR_DOWN = TFminiI2C(1, 0x10)

# Connect to the Vehicle
ground_clearance = 8
lidar_down = LIDAR_DOWN
vehicle = connect('/dev/ttyACM0', baud=57600, wait_ready=True)

# PID Controller Class
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

# PID controllers for roll, pitch, yaw, and altitude
pid_roll = PIDController(Kp=0.13, Ki=0.135, Kd=0.0036)
pid_pitch = PIDController(Kp=0.125, Ki=0.125, Kd=0.0036)
pid_yaw = PIDController(Kp=0.25, Ki=0.1, Kd=0.01)
pid_altitude = PIDController(Kp=0.4, Ki=0.2, Kd=0.1, setpoint=0.2)  # Target altitude 20 cm

def get_lidar_altitude():
    distance = lidar_down.readDistance()
    if distance is not None:
        altitude = distance - ground_clearance
        return altitude / 100
    else:
        return None

# Function to send attitude and thrust commands to the vehicle
def send_attitude_target(roll, pitch, yaw, thrust):
    msg = vehicle.message_factory.set_attitude_target_encode(
        0,  # time_boot_ms (not used)
        1,  # target system
        1,  # target component
        0b00000000,  # type mask: bit 1 is LSB
        to_quaternion(roll, pitch, yaw),  # q
        0, 0, 0,  # body roll/pitch/yaw rates
        thrust  # thrust
    )
    vehicle.send_mavlink(msg)

# Function to convert roll, pitch, yaw to quaternion
def to_quaternion(roll, pitch, yaw):
    t0 = time.time()
    t1 = 0.5 * roll
    t2 = 0.5 * pitch
    t3 = 0.5 * yaw
    t4 = 0.5 * roll
    return [t0, t1, t2, t3, t4]

# Function to stabilize the drone
def stabilize_drone():
    while True:
        # Get current roll, pitch, yaw, and altitude
        current_roll = vehicle.attitude.roll
        current_pitch = vehicle.attitude.pitch
        current_yaw = vehicle.attitude.yaw
        current_altitude = vehicle.location.global_relative_frame.alt

        # Compute control signals
        roll_control = pid_roll.compute(current_roll)
        pitch_control = pid_pitch.compute(current_pitch)
        yaw_control = pid_yaw.compute(current_yaw)
        altitude_control = pid_altitude.compute(current_altitude)

        # Send control signals to the drone
        send_attitude_target(roll_control, pitch_control, yaw_control, altitude_control)
        print(f"LiDAR Altitude: {get_lidar_altitude():.2f} m")

        time.sleep(0.1)

# Function to send velocity commands to the vehicle
def drone_velocity(velocity_x, velocity_y, velocity_z, distance):
    """
    x: pitch, y: roll, z: throttle
    """
    velocity = (velocity_x**2 + velocity_y**2 + velocity_z**2)**0.5
    if velocity == 0:
        print("Velocity cannot be zero.")
        return
    duration = distance / velocity
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        1, 1,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
        0b0000111111000111,  # type mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not used)
        0, 0)     # yaw, yaw_rate (not used)
    for _ in range(0, int(duration * 10)):
        vehicle.send_mavlink(msg)
        time.sleep(0.1)

# Function to send yaw commands to the vehicle
def yaw_control(degrees, relative=True):
    """
    + : clockwise, - : counterclockwise
    """
    if relative:
        is_relative = 1  # Yaw relative to current yaw
    else:
        is_relative = 0  # Absolute angle
    
    direction = 1 if degrees >= 0 else -1  # Clockwise or counterclockwise
    degrees = abs(degrees)  # Make the degrees positive for the command
    
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
        0,       # confirmation
        degrees, # param 1, yaw in degrees
        0,       # param 2, yaw speed deg/s
        direction,  # param 3, direction -1 ccw, 1 cw
        is_relative,  # param 4, relative offset 1, absolute angle 0
        0, 0, 0) # param 5-7 (unused)
    vehicle.send_mavlink(msg)


# Function to arm and takeoff
def arm_and_takeoff(target_altitude):
    print("Performing pre-arm checks...")
    while not vehicle.is_armable:
        print("Waiting for vehicle to initialise... Status:")
        print(f"  Is Armable: {vehicle.is_armable}")
        print(f"  GPS: {vehicle.gps_0}")
        print(f"  Battery: {vehicle.battery}")
        print(f"  EKF OK: {vehicle.ekf_ok}")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED_NOGPS")
    vehicle.armed = True

    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(target_altitude)

    while True:
        lidar_altitude = get_lidar_altitude()
        if lidar_altitude is not None:
            print(f"LiDAR Altitude: {lidar_altitude:.2f} m")
            if lidar_altitude >= target_altitude * 0.95:
                print("Reached target altitude")
                break
        time.sleep(0.1)

def land_drone():
    """
    Lands the drone.
    """
    print("Landing...")
    vehicle.mode = VehicleMode("LAND")
    while vehicle.armed:
        print(f"LiDAR Altitude: {get_lidar_altitude():.2f} m")
        time.sleep(1)
    print("Landed and disarmed")

# Function to handle bottom camera feed
def bottom_camera_feed(B_feed):
    while B_feed.isOpened():
        ret, frame = B_feed.read()
        if ret:
            lidar_altitude = get_lidar_altitude()
            if lidar_altitude is not None:
                altitude_text = f"Altitude: {lidar_altitude:.2f} m"
                cv2.putText(frame, altitude_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.imshow('Bottom Camera Feed', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            print("Failed to grab frame")
    B_feed.release()
    cv2.destroyAllWindows()

# Function to handle front camera feed
def front_camera_feed(F_feed):
    while F_feed.isOpened():
        ret, frame = F_feed.read()
        if ret:
            # Read distances from LiDAR sensors
            front_distance = LIDAR_FRONT.readDistance()
            left_distance = LIDAR_LEFT.readDistance()
            right_distance = LIDAR_RIGHT.readDistance()

            # Overlay distances on the frame
            if front_distance is not None:
                front_text = f"Front: {front_distance} cm"
                cv2.putText(frame, front_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            if left_distance is not None:
                left_text = f"Left: {left_distance} cm"
                cv2.putText(frame, left_text, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            if right_distance is not None:
                right_text = f"Right: {right_distance} cm"
                cv2.putText(frame, right_text, (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

            cv2.imshow('Front Camera Feed', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            print("Failed to grab frame")
    F_feed.release()
    cv2.destroyAllWindows()

# Main function
try:
    # Initialize camera objects
    front_camera = cv2.VideoCapture(2)  # Open the second camera
    bottom_camera = cv2.VideoCapture(0)  # Open the default camera

    # Start bottom camera feed thread
    bottom_camera_thread = threading.Thread(target=bottom_camera_feed, args=(bottom_camera,))
    bottom_camera_thread.start()

    # Start front camera feed thread
    front_camera_thread = threading.Thread(target=front_camera_feed, args=(front_camera,))
    front_camera_thread.start()

    arm_and_takeoff(0.2)  # Takeoff to 20 cm altitude
    print("Hovering and stabilizing...")

    # Start stabilize drone thread
    stabilize_drone_thread = threading.Thread(target=stabilize_drone)
    stabilize_drone_thread.start()

    time.sleep(5)
    drone_velocity(0.1, 0, 0, 1)
    time.sleep(1)
    drone_velocity(0, 0.1, 0, 1)
    time.sleep(1)
    yaw_control(90)

    time.sleep(30)  # Hover for 30 seconds
    land_drone()
except KeyboardInterrupt:
    print("Interrupted by user, landing the drone")
    land_drone()
    vehicle.close()
finally:
    print("Closing vehicle connection...")
    vehicle.close()
    # Ensure camera threads are properly closed
    if bottom_camera_thread.is_alive():
        bottom_camera.release()
        cv2.destroyAllWindows()
    if front_camera_thread.is_alive():
        front_camera.release()
        cv2.destroyAllWindows()
