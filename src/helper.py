# helper functions used for image processing and contour detection. universal across obstacle and open challenge
import cv2
import numpy as np
import subprocess
from CONSTS import *

def pwm(degree):
    """convert degree to pulse width modulation signal"""
    pw = round(degree * 11.1 + 1500)
    return max(0, min(65535, pw))


def sortContourShapes(contours):
    """sort contours by area"""
    if not contours:
        return [None]
    contours_sorted = sorted(contours, key=cv2.contourArea, reverse=True)
    return contours_sorted

def is_valid_contour(c):
    """check if contour exists"""
    return c is not None and isinstance(c, (list, tuple, np.ndarray)) and len(c) > 0

def drawContour(contour, roi, frame, color=(0, 255, 0), thickness=2):
    """draw contour on frame"""
    if is_valid_contour(contour):
        cv2.drawContours(frame, [contour], -1, color, thickness)

def drawRect(roi, frame, color=(0, 255, 0), thickness=2):
    """draw rectangle on frame"""
    if is_valid_contour(roi):
        cv2.rectangle(frame, (roi[0], roi[1]), (roi[2], roi[3]), color, thickness)


def listen_to_button_events():
    """Listen for button events from the ROS 2 topic."""
    button_id = 0
    command = 'source /home/ubuntu/.zshrc && ros2 topic echo /ros_robot_controller/button'
    process = subprocess.Popen(
        ['docker', 'exec', '-u', 'ubuntu', '-w', '/home/ubuntu', 'MentorPi', '/bin/zsh', '-c', command],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True
    )

    while True:
        output = process.stdout.readline()
        if output:
            line = output.strip()
            print("WAITINGGGGGG")
            if line.startswith("id:"):
                button_id = int(line.split(":")[1].strip())
            elif line.startswith("state:"):
                state = int(line.split(":")[1].strip())

                if button_id == 1:
                    if state == 1:
                        print("Button 1 pressed")
                elif button_id == 2:
                    if state == 1:
                        print("Button 2 pressed")
                        return
        else:
            continue

def check_node_status():
    """Check the status of the ROS 2 node."""
    command = 'source /home/ubuntu/.zshrc && ros2 topic list'
    result = subprocess.run(['docker', 'exec', '-u', 'ubuntu', '-w', '/home/ubuntu', 'MentorPi', '/bin/zsh', '-c', command], capture_output=True, text=True)
    res = result.stdout
    return '/ros_robot_controller/button' in res

def findContours(img_lab, lab_range, ROI=None, test=False):
    """Find contours in the image based on LAB color range."""
    # Extract ROI or full image
    if ROI:
        x1, y1, x2, y2 = ROI
        if x1 >= x2 or y1 >= y2:
            return []
        img_segmented = img_lab[y1:y2, x1:x2]
        if img_segmented is None or img_segmented.size == 0:
            return []
    else:
        img_segmented = img_lab

    # Create mask based on LAB range
    lower_mask = np.array(lab_range[0], dtype=np.uint8)
    upper_mask = np.array(lab_range[1], dtype=np.uint8)
    mask = cv2.inRange(img_segmented, lower_mask, upper_mask)

    # Morphological operations
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=1)

    # Find contours
    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

    # If ROI, adjust contour coordinates
    if ROI:
        adjusted_contours = []
        for contour in contours:
            contour[:, :, 0] += x1
            contour[:, :, 1] += y1
            adjusted_contours.append(contour)
    else:
        adjusted_contours = contours

    if test:
        return adjusted_contours, mask
    return adjusted_contours


def LED(board, colour):
    """Set the LED colour on the board."""
    board.set_rgb([[1, *colour], [2, *colour]])

def preprocess_frame(frame):
    """Preprocess the input frame for contour detection."""
    # shift mask to match room lighting
    frame_lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB).astype(np.float32)
    frame_lab += np.array(SHIFT_FROM_MASK)
    frame_lab = np.clip(frame_lab, 0, 255).astype(np.uint8)

    # convert to LAB color space
    corrected_frame = cv2.cvtColor(frame_lab, cv2.COLOR_LAB2BGR)
    frame_processed = cv2.GaussianBlur(corrected_frame, (7, 7), 0)
    frame_processed = cv2.cvtColor(frame_processed, cv2.COLOR_BGR2LAB)
    return frame_processed
    