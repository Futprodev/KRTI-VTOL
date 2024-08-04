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
            # If altitude exceeds target, lower the throttle
            elif lidar_altitude > target_altitude:
                print("Altitude exceeded target, lowering throttle")
                vehicle.channels.overrides['3'] = 1300  # Lower throttle
        time.sleep(0.1)

    # Clear any channel overrides
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

# Main function
try:
    if not bottom_cam.isOpened():
        print("Error: Could not open camera.")
        exit()
        
    # Start the camera feed in a separate thread
    camera_thread = threading.Thread(target=bottom_feed, args=(bottom_cam,))
    camera_thread.start()

    arm_and_takeoff(0.2) 
    print("Hovering for 5 seconds...")
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
