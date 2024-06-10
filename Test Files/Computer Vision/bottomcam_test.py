import numpy as np
import cv2

# Define the lower and upper bounds of orange
lower_bound_orange = np.array([4, 120, 60])
upper_bound_orange = np.array([23, 255, 255])

# Open the camera
cap = cv2.VideoCapture(0)

# Initialize variables to store the center coordinates
last_cX, last_cY = None, None

while cap.isOpened():
    # Read a frame from the video and convert it to HSV
    success, frame = cap.read()
    if not success:
        break
    
    frame_height, frame_width = frame.shape[:2]
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create a binary mask for orange
    mask_orange = cv2.inRange(hsv, lower_bound_orange, upper_bound_orange)

    contours_orange, _ = cv2.findContours(mask_orange, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours_orange:
        # Find the largest contour by area
        largest_contour = max(contours_orange, key=cv2.contourArea)

        # Get the moments of the largest contour
        M = cv2.moments(largest_contour)

        if M["m00"] != 0:
            # Calculate the center of the contour
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])

            # Update the last known center coordinates
            last_cX, last_cY = cX, cY
            
            # Draw the largest contour
            cv2.drawContours(frame, [largest_contour], -1, (0, 255, 0), 2)

            # Gerakan drone
            # Center coordinates (INI GANTI SESUAI KOORDINAT KAMERA)
            drop_X = [int(frame_width / 2) - 200, int(frame_width / 2) + 200]
            drop_Y = [int(frame_height / 2) - 200, int(frame_height / 2) + 200]
            cv2.rectangle(frame, (drop_X[0], drop_Y[0]), (drop_X[1], drop_Y[1]), (255, 0, 0), 2)

            # Check if the drone is centered
            X_aman = drop_X[0] <= last_cX <= drop_X[1]
            Y_aman = drop_Y[0] <= last_cY <= drop_Y[1]

            if not X_aman:
                if last_cX < drop_X[0]:
                    print("Gerak ke kanan")
                else:
                    print("Gerak ke kiri")

            if not Y_aman:
                if last_cY < drop_Y[0]:
                    print("Gerak ke belakang")
                else:
                    print("Gerak ke depan")

            if X_aman and Y_aman:
                cv2.putText(frame, "TURUN :D", (600, 600), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

            # Ensure the center dot is always drawn
            if last_cX is not None and last_cY is not None:
                cv2.circle(frame, (last_cX, last_cY), 7, (255, 0, 0), -1)
                cv2.putText(frame, f"Center: ({last_cX}, {last_cY})", (last_cX - 50, last_cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    # Display the frames
    cv2.imshow('Orange', mask_orange)
    #cv2.imshow('Edge', edges_orange)
    cv2.imshow('Bottom Cam', frame)

    # Break the loop if 'q' key is pressed (UTK MAC)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
