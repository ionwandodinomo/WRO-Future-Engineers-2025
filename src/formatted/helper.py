#listen_to_button_events,  check_node_status
import cv2
import numpy as np
import subprocess

def pwm(degree):  
    pw = round(degree * 11.1 + 1500)
    return max(0, min(65535, pw))


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
        contour[:, :, 0] += roi[0]
        contour[:, :, 1] += roi[1]
        cv2.drawContours(frame, [contour], -1, color, thickness)

def drawRect(roi, frame, color=(0, 255, 0), thickness=2):
    if is_valid_contour(roi):
        cv2.rectangle(frame, (roi[0], roi[1]), (roi[2], roi[3]), color, thickness)


def listen_to_button_events():
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

def findContours(img_lab, lab_range, ROI):
    
    x1, y1, x2, y2 = ROI
    if x1 >= x2 or y1 >= y2:  # invalid ROI
        return []

    img_segmented = img_lab[y1:y2, x1:x2]
    if img_segmented is None or img_segmented.size == 0:
        return []
    
    #segment image to only be the ROI
    img_segmented = img_lab[ROI[1]:ROI[3], ROI[0]:ROI[2]]
    
    lower_mask = np.array(lab_range[0])
    upper_mask = np.array(lab_range[1])

    #threshold image
    mask = cv2.inRange(img_segmented, lower_mask, upper_mask)
    
    kernel = np.ones((5, 5), np.uint8)
    
    #perform erosion and dilation
    eMask = cv2.erode(mask, kernel, iterations=1)
    dMask = cv2.dilate(eMask, kernel, iterations=1)
    
    #find contours
    contours = cv2.findContours(dMask, cv2.RETR_EXTERNAL,
    cv2.CHAIN_APPROX_SIMPLE)[-2]
    
    return contours

