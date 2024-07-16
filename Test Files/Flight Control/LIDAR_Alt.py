from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import threading

# Connect to the Vehicle
vehicle = connect('/dev/ttyAMA0', baud=57600, wait_ready=True)

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

def send_velocity(vehicle, velocity_x, velocity_y, velocity_z):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_BODY_NED, 0b0000111111000111,
        0, 0, 0, velocity_x, velocity_y, velocity_z, 0, 0, 0, 0, 0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

def maintain_altitude(target_altitude):
    while True:
        current_altitude = get_lidar_altitude()
        print(f"Current LiDAR Altitude: {current_altitude} meters")
        
        if current_altitude < target_altitude - 0.1:
            send_velocity(vehicle, 0, 0, 0.1)  # Ascend
        elif current_altitude > target_altitude + 0.1:
            send_velocity(vehicle, 0, 0, -0.1)  # Descend
        else:
            send_velocity(vehicle, 0, 0, 0)  # Hold position
        
        time.sleep(0.1)

def main():
    target_altitude = 2.0  # Target altitude in meters
    arm_and_takeoff(target_altitude)
    
    altitude_thread = threading.Thread(target=maintain_altitude, args=(target_altitude,))
    altitude_thread.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Landing...")
        vehicle.mode = VehicleMode("LAND")
        altitude_thread.join()
        vehicle.close()

if __name__ == "__main__":
    main()