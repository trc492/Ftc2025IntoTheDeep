import cv2
import numpy as np

def runPipeline(image, llrobot):
    # Initialize variables to avoid unboundlocal errors
    largestContour = np.array([[]])
    llpython = [0, 0, 0, 0, 0, 0, 0, 0]
    x, y, w, h = 0, 0, 0, 0

    # Convert BGR to YCrCb
    ycrcb_image = cv2.cvtColor(image, cv2.COLOR_BGR2YCrCb)

    # Define range for yellow color in YCrCb
    lower_yellow = np.array([100, 120, 30])
    upper_yellow = np.array([250, 200, 80])

    # Create a mask for yellow color
    mask = cv2.inRange(ycrcb_image, lower_yellow, upper_yellow)

    # Reduce noise in the mask
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        # Find the largest contour
        largestContour = max(contours, key=cv2.contourArea)

        # Get bounding box of the largest contour
        x, y, w, h = cv2.boundingRect(largestContour)

        # Draw the contour and bounding box
        cv2.drawContours(image, [largestContour], 0, (0, 255, 0), 2)
        cv2.rectangle(image, (x, y), (x+w, y+h), (255, 0, 0), 2)

        # Calculate center of the bounding box
        center_x = x + w // 2
        center_y = y + h // 2

        # Draw crosshair at the center
        cv2.line(image, (center_x - 10, center_y), (center_x + 10, center_y), (0, 0, 255), 2)
        cv2.line(image, (center_x, center_y - 10), (center_x, center_y + 10), (0, 0, 255), 2)

        # Prepare data to send back to the robot
        area = cv2.contourArea(largestContour)
        llpython = [1, center_x, center_y, w, h, area, 0, 0]

    # Add text to the image
    cv2.putText(image, "Yellow Sample Detector", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

    # If a yellow sample is detected, display its information
    if llpython[0] == 1:
        info_text = f"Center: ({llpython[1]}, {llpython[2]}), Size: {llpython[3]}x{llpython[4]}, Area: {llpython[5]}"
        cv2.putText(image, info_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    return largestContour, image, llpython
