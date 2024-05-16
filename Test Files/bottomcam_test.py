import numpy as np
import cv2

#OPEN CAMERA
cap = cv2.VideoCapture(0)

while cap.isOpened():
    # Read a frame from the video and convert it to HSV
    success, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define the lower and upper bounds of orange
    lower_bound_orange = np.array([5, 80, 80])
    upper_bound_orange = np.array([9, 255, 255])

    # Createq a binary mask for orange
    mask_orange = cv2.inRange(hsv, lower_bound_orange, upper_bound_orange)

    # Apply the mask to the original image
    result_orange = cv2.bitwise_and(frame, frame, mask=mask_orange)

    # Blur the result to reduce noise to detect edges & contours
    blurred_orange = cv2.GaussianBlur(result_orange, (9, 9), 0)
    edges_orange = cv2.Canny(blurred_orange, 50, 150)
    contours_orange, _ = cv2.findContours(edges_orange, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Filter contours based on area (example: exclude small contours)
    min_contour_area = 1000
    filtered_contours_orange = [cnt for cnt in contours_orange if cv2.contourArea(cnt) > min_contour_area]
    
    # Draw contours on the original image
    cv2.drawContours(frame, filtered_contours_orange, -1, (0, 255, 0), 2)

    cv2.imshow('HSV', hsv)
    cv2.imshow('Orange', result_orange)
    cv2.imshow('Bottom Cam', frame)

    # Break the loop if 'q' key is pressed (KHUSUS MAC)
    if cv2.waitKey(1) & 0xFF == ord("q"):
            break
cap.release()
cv2.destroyAllWindows()