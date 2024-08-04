import time
from dronekit import connect, VehicleMode
from pymavlink import mavutil
import threading

# Connect to the Vehicle
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
pid_roll = PIDController(Kp=0.2, Ki=0.1, Kd=0.01)
pid_pitch = PIDController(Kp=0.2, Ki=0.1, Kd=0.01)
pid_yaw = PIDController(Kp=0.25, Ki=0.1, Kd=0.01)
pid_altitude = PIDController(Kp=0.4, Ki=0.2, Kd=0.1, setpoint=0.2)  # Target altitude 20 cm

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
    vehicle.flush()

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

        time.sleep(0.1)

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
        time.sleep(0.1)

# Main function
try:
    arm_and_takeoff(0.2)  # Takeoff to 20 cm altitude
    print("Hovering and stabilizing...")
    stabilize_drone_thread = threading.Thread(target=stabilize_drone)
    stabilize_drone_thread.start()
    time.sleep(30)  # Hover for 30 seconds
    land_drone()
except KeyboardInterrupt:
    print("Interrupted by user, landing the drone")
    land_drone()
finally:
    print("Closing vehicle connection...")
    vehicle.close()
    bottom_cam.release()
    cv2.destroyAllWindows()
