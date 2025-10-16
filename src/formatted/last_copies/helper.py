#listen_to_button_events,  check_node_status
import cv2
import numpy as np
import subprocess
import rclpy
from CONSTS import *
from imu_node import *
def pwm(degree):  
    pw = round(degree * 11.1 + 1500)
    return max(0, min(65535, pw))

def start_nodes(IMU = False, yes = False):
    if yes:
        commands = [
            "ros2 run ros_robot_controller ros_robot_controller",
            "ros2 launch ros_robot_controller ros_robot_controller.launch.py",
        ]

        for cmd in commands:
            docker_cmd = (
                f"source /home/ubuntu/.zshrc && {cmd}"
            )
            process = subprocess.Popen(
                [
                    "docker", "exec",
                    "-u", "ubuntu",
                    "-w", "/home/ubuntu",
                    "MentorPi",
                    "/bin/zsh", "-c", docker_cmd
                ],
                start_new_session=True
            )

    
    if IMU:
        rclpy.init()

        # Create IMU node
        imu = create_imu_node()
        rclpy.spin(imu)
        imu.destroy_node()
        rclpy.shutdown()


"""def findMaxContourShape(contours):
    if contours is None:
        raise Exception
    max_area = 0
    max_contour = None
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > max_area:
            max_area = area
            max_contour = cnt
    return max_contour, max_area"""

"""def findSecondMaxContourShape(contours):
    if not contours or len(contours) < 2:
        return None, 0 

    contours_sorted = sorted(contours, key=cv2.contourArea, reverse=True)

    second_max_contour = contours_sorted[1]
    second_max_area = cv2.contourArea(second_max_contour)

    return second_max_contour, second_max_area"""

def sortContourShapes(contours):
    if not contours:
        return [None]
    contours_sorted = sorted(contours, key=cv2.contourArea, reverse=True)
    return contours_sorted

def is_valid_contour(c):
    return c is not None and isinstance(c, (list, tuple, np.ndarray)) and len(c) > 0

def drawContour(contour, roi, frame, color=(0, 255, 0), thickness=2):
    if is_valid_contour(contour):
        #contour[:, :, 0] += roi[0]
        #contour[:, :, 1] += roi[1]
        cv2.drawContours(frame, [contour], -1, color, thickness)

def drawRect(roi, frame, color=(0, 255, 0), thickness=2):
    if is_valid_contour(roi):
        cv2.rectangle(frame, (roi[0], roi[1]), (roi[2], roi[3]), color, thickness)


def listen_to_button_events(board):
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
            #print("WAITINGGGGGG")
            LED(board,(0,255,0))
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
                        process.terminate()  # sends SIGTERM (asks it to exit cleanly)
                        process.kill()       # sends SIGKILL (forces it to die)
                        return
        else:
            continue

def check_node_status():
    command = 'source /home/ubuntu/.zshrc && ros2 topic list'
    result = subprocess.run(['docker', 'exec', '-u', 'ubuntu', '-w', '/home/ubuntu', 'MentorPi', '/bin/zsh', '-c', command], capture_output=True, text=True)
    res = result.stdout
    return '/ros_robot_controller/button' in res


"""def findContours(thresh,roi):
    contours, hierarchy = cv2.findContours(
        thresh[
            roi[1] : roi[3], roi[0] : roi[2]
        ],
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_NONE,
    )
    return contours, hierarchy"""
"""
def findContours(img_lab, lab_range, ROI=None,test=False):
    
    if ROI:
        x1, y1, x2, y2 = ROI
        if x1 >= x2 or y1 >= y2: 
            return []
        
        img_segmented = img_lab[y1:y2, x1:x2]
        if img_segmented is None or img_segmented.size == 0:
            return []

        img_segmented = img_lab[ROI[1]:ROI[3], ROI[0]:ROI[2]]
    else:
        img_segmented = img_lab
    
    lower_mask = np.array(lab_range[0])
    upper_mask = np.array(lab_range[1])


    mask = cv2.inRange(img_segmented, lower_mask, upper_mask)
    
    kernel = np.ones((5, 5), np.uint8)

    eMask = cv2.erode(mask, kernel, iterations=1)
    dMask = cv2.dilate(eMask, kernel, iterations=1)
    
    #find contours
    contours = cv2.findContours(dMask, cv2.RETR_EXTERNAL,
    cv2.CHAIN_APPROX_SIMPLE)[-2]
    
    if ROI:
        contour2 = []
        for contour in contours:
            contour[:, :, 0] += ROI[0]
            contour[:, :, 1] += ROI[1]
            contour2.append(contour)
    else:
        contour2 = contours
    if test:
        return contour2, dMask
    return contour2"""
def findContours(img_lab, lab_range, ROI=None, test=False):
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

"""
def findContours(img_lab, lab_range, ROI):
    x1, y1, x2, y2 = ROI
    if x1 >= x2 or y1 >= y2:
        return []

    img_segmented = img_lab[y1:y2, x1:x2]
    if img_segmented is None or img_segmented.size == 0:
        return []

    lower_mask = np.array(lab_range[0], dtype=np.uint8)
    upper_mask = np.array(lab_range[1], dtype=np.uint8)

    # Apply CLAHE on L channel
    L, A, B = cv2.split(img_segmented)
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
    L = clahe.apply(L)
    img_segmented = cv2.merge([L, A, B])

    # Gaussian blur for noise reduction
    img_segmented = cv2.GaussianBlur(img_segmented, (7, 7), 0)

    # Threshold based on LAB range
    mask = cv2.inRange(img_segmented, lower_mask, upper_mask)

    # Morphological operations
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.dilate(cv2.erode(mask, kernel), kernel)

    # Find contours
    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

    # Adjust contours to global ROI coordinates
    adjusted_contours = []
    for c in contours:
        c[:, :, 0] += ROI[0]
        c[:, :, 1] += ROI[1]
        adjusted_contours.append(c)

    return adjusted_contours
"""
def flatten_colors(img, K=8):
    Z = img.reshape((-1, 3))
    Z = np.float32(Z)

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
    _, labels, centers = cv2.kmeans(Z, K, None, criteria, 1, cv2.KMEANS_RANDOM_CENTERS)

    centers = np.uint8(centers)
    flattened = centers[labels.flatten()]
    return flattened.reshape(img.shape)

def LED(board, colour):
    board.set_rgb([[1, *colour], [2, *colour]])

def preprocess_frame(frame):
    frame_lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB).astype(np.float32)
    frame_lab += np.array(SHIFT_FROM_MASK)
    frame_lab = np.clip(frame_lab, 0, 255).astype(np.uint8)

    corrected_frame = cv2.cvtColor(frame_lab, cv2.COLOR_LAB2BGR)
    #flattened_frame = flatten_colors(corrected_frame)
    frame_processed = cv2.GaussianBlur(corrected_frame, (7, 7), 0)
    frame_processed = cv2.cvtColor(frame_processed, cv2.COLOR_BGR2LAB)
    return frame_processed
    
