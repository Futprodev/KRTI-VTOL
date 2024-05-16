import numpy as np
import cv2


def get_center_coordinates(contour):
    x, y, w, h = cv2.boundingRect(contour)
    center_x = x + w // 2
    center_y = y + h // 2
    return (center_x, center_y)


#OPEN CAMERA
cap = cv2.VideoCapture(0)

while cap.isOpened():
    # Read a frame from the video and convert it to HSV
    success, frame = cap.read()
    if not success:
        break

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)


    # ORANGE

    # Define the lower and upper bounds of orange
    lower_bound_orange = np.array([5, 60, 60])
    upper_bound_orange = np.array([9, 255, 255])

    # Create a binary mask for orange
    mask_orange = cv2.inRange(hsv, lower_bound_orange, upper_bound_orange)

    # Apply the mask to the original image
    result_orange = cv2.bitwise_and(frame, frame, mask=mask_orange)

    # Blur the result to reduce noise to detect edges & contours
    blurred_orange = cv2.GaussianBlur(result_orange, (9, 9), 0)
    edges_orange = cv2.Canny(blurred_orange, 50, 150)
    contours_orange, _ = cv2.findContours(edges_orange, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours_orange:
        # Find the largest contour by area
        largest_contour = max(contours_orange, key=cv2.contourArea)

        # Get the moments of the largest contour
        M = cv2.moments(largest_contour)

        if M["m00"] != 0:
            # Calculate the center of the contour
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            
            # Draw the largest contour
            cv2.drawContours(frame, [largest_contour], -1, (0, 255, 0), 2)
            # Draw a dot at the center of the largest contour
            cv2.circle(frame, (cX, cY), 7, (255, 0, 0), -1)
            # Add text to show the coordinates
            cv2.putText(frame, f"Center: ({cX}, {cY})", (cX - 50, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            # Gerakan drone
            # Center coordinates (INI GANTI SESUAI KOORDINAT KAMERA)
            drop_X = [450, 850]
            drop_Y = [150, 550]
            cv2.rectangle(frame, [drop_X[0], drop_Y[0]], [drop_X[1], drop_Y[1]], (255, 0, 0), 2)

            # Check if the drone is centered
            X_aman = False
            Y_aman = False

            # Drone harus gerak ke mana
            if cX < drop_X[0]:
                 X_aman = False
                 print("Gerak ke kanan")
            elif cX > drop_X[1]:
                 X_aman = False
                 print("Gerak ke kiri")
            else:
                 X_aman = True

            if cY < drop_Y[0]:
                 Y_aman = False
                 print("Gerak ke belakang")
            elif cY > drop_X[1]:
                 Y_aman = False
                 print("Gerak ke depan")
            else:
                 Y_aman = True
            
            # Boleh turun
            if X_aman == True and Y_aman == True:
                 cv2.putText(frame, "TURUN :D", (600, 600), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

    #Display
    cv2.imshow('HSV', hsv)
    cv2.imshow('Orange', result_orange)
    cv2.imshow('Bottom Cam', frame)

    # Break the loop if 'q' key is pressed (UTK MAC)
    if cv2.waitKey(1) & 0xFF == ord("q"):
            break
cap.release()
cv2.destroyAllWindows()