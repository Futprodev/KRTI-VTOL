import time
from smbus2 import SMBus, i2c_msg
from dronekit import connect, VehicleMode
import cv2
import threading

bottom_cam = cv2.VideoCapture(0)

# LiDAR class
class TFminiI2C:
    def __init__(self, I2Cbus, address):
        self.I2Cbus = I2Cbus
        self.address = address

    def readDistance(self):
        try:
            write = i2c_msg.write(self.address, [1, 2, 7])
            read = i2c_msg.read(self.address, 7)
            with SMBus(self.I2Cbus) as bus:
                bus.i2c_rdwr(write, read)
                data = list(read)
                dist = data[3] << 8 | data[2]
            return dist
        except Exception as e:
            print(f"Error reading from LiDAR at address {hex(self.address)}: {e}")
            return None

# Setup bottom LiDAR
LIDAR_DOWN = TFminiI2C(1, 0x10)
GROUND_CLEARANCE = 9

def get_lidar_altitude():
    distance = LIDAR_DOWN.readDistance()
    if distance is not None:
        altitude = distance - GROUND_CLEARANCE
        return altitude / 100.0  
    else:
        return None

# Connect to the Vehicle
vehicle = connect('/dev/ttyACM0', baud=57600, wait_ready=True)

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

pid_z = PIDController(Kp=1.0, Ki=0.1, Kd=0.05, setpoint=0.2)  # Target altitude of 0.2 meters

def move_drone(forward_speed, side_speed, z_speed=0):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_BODY_NED, 0b0000111111000111,
        0, 0, 0, forward_speed, side_speed, z_speed, 0, 0, 0, 0, 0
    )
    vehicle.send_mavlink(msg)

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
    vehicle.mode = VehicleMode("GUIDED")
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
            elif lidar_altitude > target_altitude:
                print("Altitude exceeded target, adjusting throttle")
                z_speed = pid_z.compute(lidar_altitude)
                move_drone(0, 0, z_speed)
        time.sleep(0.1)

    vehicle.channels.overrides = {}

def land_drone():
    print("Landing...")
    vehicle.mode = VehicleMode("LAND")
    while vehicle.armed:
        lidar_altitude = get_lidar_altitude()
        if lidar_altitude is not None:
            print(f"LiDAR Altitude: {lidar_altitude:.2f} m")
        time.sleep(1)
    print("Landed and disarmed")

def bottom_feed(cam):
    while cam.isOpened():
        ret, frame = cam.read()
        if ret:
            cv2.imshow('Bottom Camera Feed', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            print("Failed to capture video")
            break
    cam.release()
    cv2.destroyAllWindows()

def maintain_position_and_altitude(target_altitude):
    pid_x = PIDController(Kp=1.0, Ki=0.1, Kd=0.05, setpoint=0)
    pid_y = PIDController(Kp=1.0, Ki=0.1, Kd=0.05, setpoint=0)

    while True:
        lidar_altitude = get_lidar_altitude()
        if lidar_altitude is not None:
            z_speed = pid_z.compute(lidar_altitude)
        else:
            z_speed = 0

        move_drone(0, 0, z_speed)
        time.sleep(0.1)

try:
    if not bottom_cam.isOpened():
        print("Error: Could not open camera.")
        exit()

    camera_thread = threading.Thread(target=bottom_feed, args=(bottom_cam,))
    camera_thread.start()

    arm_and_takeoff(0.2)
    print("Hovering for 5 seconds...")
    maintain_position_thread = threading.Thread(target=maintain_position_and_altitude, args=(0.2,))
    maintain_position_thread.start()
    time.sleep(5)
    land_drone()
except KeyboardInterrupt:
    print("Interrupted by user, stopping motor test")
    land_drone()
finally:
    print("Closing vehicle connection...")
    vehicle.close()
    bottom_cam.release()
    cv2.destroyAllWindows()
