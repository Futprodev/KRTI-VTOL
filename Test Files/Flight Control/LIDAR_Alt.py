import time
from smbus2 import SMBus, i2c_msg
from dronekit import connect, VehicleMode

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
GROUND_CLEARANCE = 17 

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
        lidar_altitude = get_lidar_altitude()
        if lidar_altitude is not None:
            print(f"LiDAR Altitude: {lidar_altitude:.2f} m")
            if lidar_altitude >= target_altitude * 0.95:
                print("Reached target altitude")
                break
        time.sleep(1)

def land_drone():
    print("Landing...")
    vehicle.mode = VehicleMode("LAND")
    while vehicle.armed:
        print(f"LiDAR Altitude: {get_lidar_altitude():.2f} m")
        time.sleep(1)
    print("Landed and disarmed")

# Main function
def main():
    try:
        arm_and_takeoff(0.2) 
        print("Hovering for 5 seconds...")
        time.sleep(5)  
        land_drone() 
    finally:
        print("Closing vehicle connection...")
        vehicle.close()

# Run the main function
if __name__ == "__main__":
    main()
