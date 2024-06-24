import numpy as np
import cv2
import time

# Define the lower and upper bounds of orange and red
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

def drone_centering(center, frame_center, pid_x, pid_y):
    # Calculate the difference between the center of the frame and the center of the object
    x_diff = center[0] - frame_center[0]
    y_diff = center[1] - frame_center[1]
    vx = pid_x.compute(x_diff) * 0.001 # adjust pake drone
    vy = pid_y.compute(y_diff) * 0.001 # adjust pake drone

    # Limit the velocities to a max value
    max_vel = 0.5 # adjust pake drone
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
        # Proportional term
        P_out = self.Kp * error
        # Integral term
        self.integral += error * delta_time
        I_out = self.Ki * self.integral
        # Derivative term
        delta_error = error - self.last_error
        D_out = self.Kd * (delta_error / delta_time) if delta_time > 0 else 0
        # Total output
        output = P_out + I_out + D_out
        # Save error and time for the next calculation
        self.last_error = error
        self.last_time = current_time
        return output


# Drone lagi ngapain?
stage = input("Type 1/2/3 \n 1: Cari barang orange \n 2: Barang orange udh diangkat, cari ember merah \n 3: Idle \n")

# PID controllers for x and y directions
pid_x = PIDController(Kp=1.0, Ki=0.1, Kd=0.05, setpoint=0)
pid_y = PIDController(Kp=1.0, Ki=0.1, Kd=0.05, setpoint=0)

cap = cv2.VideoCapture(0)
while cap.isOpened():
    success, frame = cap.read()
    if not success:
        break

    if stage == "1":
        contours = detect_orange(frame)
    elif stage == "2":
        contours = detect_red(frame)
    else:
        contours = None

    frame_height, frame_width = frame.shape[:2]
    frame_center = [frame_width // 2, frame_height // 2] # Ganti ini ke target koordinat
    cv2.circle(frame, (frame_center[0], frame_center[1]), 7, (255, 0, 255), -1)
    cv2.putText(frame, f" Frame Center: ({frame_center[0]}, {frame_center[1]})", (frame_center[0] - 70, frame_center[1] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    if contours:
        # Get the largest contour
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

        # Velocity
        velocity_x, velocity_y = drone_centering(center, frame_center, pid_x, pid_y)
        print(f"Velocity X: {velocity_x:.2f}, Velocity Y: {velocity_y:.2f}")

    # Display the frames
    cv2.imshow('Bottom Cam', frame)

    # Break the loop if 'q' key is pressed (UTK MAC)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
