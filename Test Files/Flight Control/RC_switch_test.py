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

def return_to_launch():
    print("Returning to Launch")
    vehicle.mode = VehicleMode("RTL")

def set_rc_mode():
    vehicle.mode = VehicleMode("LOITER")
    print("Switched to RC control mode")

def set_autonomous_mode():
    vehicle.mode = VehicleMode("GUIDED")
    print("Switched to Autonomous control mode")

def check_signal_strength():
    return vehicle.last_heartbeat

def read_rc_channel(channel):
    return vehicle.channels[channel]

def check_rc_switch():
    rc_value = read_rc_channel(7) #change according to rc5
    print(f"RC Channel 7 value: {rc_value}")  # Print the actual RC channel value for debugging
    if rc_value > 1750:
        set_rc_mode()  # Switch to RC control mode
    elif 1250 < rc_value <= 1750:
        return_to_launch()  # Switch to Return to Launch mode
    elif rc_value <= 1250:
        set_autonomous_mode()  # Switch to Autonomous mode

# Main function
def main():
    global mode
    mode = "auto"

    try:
        arm_and_takeoff(0.2) 
        print("Hovering for 10 seconds...")
        
        start_time = time.time()
        
        while True:
            signal_strength = check_signal_strength()
            if signal_strength is None or signal_strength < 1:
                print("Signal lost or weak. Initiating return to launch...")
                return_to_launch()
                break
            
            check_rc_switch()  # Check the RC switch position
            
            # Check if the current mode is RC control mode
            if vehicle.mode.name == "STABILIZE":
                print("RC control mode active. Not landing.")
                continue

            if time.time() - start_time > 10:  # Hover for 5 seconds
                break
            
            time.sleep(1)

        # Only land if the mode is not RC control mode
        if vehicle.mode.name != "STABILIZE":
            land_drone()
    finally:
        print("Closing vehicle connection...")
        vehicle.close()

if __name__ == "__main__":
    main()
