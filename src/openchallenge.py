import rclpy
import cv2
import numpy as np
from picamera2 import Picamera2
import ros_robot_controller_sdk as rcc
from helper import *
from CONSTS import *
import time
board = rcc.Board()

# reconfigured consts for open challenge
WALL_THRESH = 200
ROI_LEFT_TOP = [0, 230, 225, 290]        
ROI_RIGHT_TOP = [415, 230, 640, 290]
ROI_LEFT_BOT = [0, 290, 115, 315]
ROI_RIGHT_BOT = [525, 290, 640, 315]

# dynamic variables initialization
speed = 1615
last_diff = 0
turning = False
turn_dir = 0
track_dir = 0
turn_count = 0
max_turns = 12

actions_to_straight = 0

debug = True

if check_node_status():
    """run code if button pressed"""
    print("READy")
    LED(board,(0,255,0))
    listen_to_button_events()
    print("STARTING OPEN CHALLENGE CODE")
    LED(board,(255,255,255))
    time.sleep(1)
    LED(board,(0,0,0))
    time.sleep(0.2)
    LED(board,(255,255,255))
    time.sleep(1)
    LED(board,(0,0,0))
    time.sleep(0.2)
    LED(board,(0,255,0))
    time.sleep(1)
    LED(board,(0,0,0))


def clamp_to_max(angle):
    """clamp servo angle to maximum turn degree"""
    if angle >0:
        return min(angle, MAX_TURN_DEGREE)
    return max(angle, -MAX_TURN_DEGREE)

# configure camera settings
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640,480)
picam2.preview_configuration.main.format = "RGB888"
print(picam2.preview_configuration.controls.FrameRate)
picam2.preview_configuration.controls.FrameRate = 75
picam2.set_controls({"Brightness": 0.05})
print(picam2.preview_configuration.controls.FrameRate)
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

# control loop
while True:
    # capture frame from camera
    frame = picam2.capture_array()
    frame = cv2.resize(frame, (640, 480))


    img_lab = cv2.cvtColor(frame, cv2.COLOR_BGR2Lab)

    # extract contours for walls
    left_contours_top = findContours(img_lab, rBlack, ROI_LEFT_TOP)
    right_contours_top = findContours(img_lab, rBlack, ROI_RIGHT_TOP)
    left_contours_bot = findContours(img_lab, rBlack, ROI_LEFT_BOT)
    right_contours_bot = findContours(img_lab, rBlack, ROI_RIGHT_BOT)

    # extract line contours
    contours_blue1 = findContours(img_lab, rBlue, ROI_LINE1)
    contours_orange1 = findContours(img_lab, rOrange, ROI_LINE1)
    contours_blue2 = findContours(img_lab, rBlue, ROI_LINE2)
    contours_orange2 = findContours(img_lab, rOrange, ROI_LINE2)

    # find max contours of walls
    max_left_top_contour = sortContourShapes(left_contours_top)[0]
    left_area_top = cv2.contourArea(max_left_top_contour) if is_valid_contour(max_left_top_contour) else 0

    max_right_top_contour = sortContourShapes(right_contours_top)[0]
    right_area_top = cv2.contourArea(max_right_top_contour) if is_valid_contour(max_right_top_contour) else 0

    max_left_bot_contour = sortContourShapes(left_contours_bot)[0]
    left_area_bot = cv2.contourArea(max_left_bot_contour) if is_valid_contour(max_left_bot_contour) else 0

    max_right_bot_contour = sortContourShapes(right_contours_bot)[0]
    right_area_bot = cv2.contourArea(max_right_bot_contour) if is_valid_contour(max_right_bot_contour) else 0

    right_area = right_area_bot + right_area_top
    left_area = left_area_bot + left_area_top

    # find max contours of lines
    max_blue_contour1 = sortContourShapes(contours_blue1)[0] if contours_blue1 else None
    max_blue_area1 = cv2.contourArea(max_blue_contour1) if is_valid_contour(max_blue_contour1) else 0
    max_orange_contour1 = sortContourShapes(contours_orange1)[0] if contours_orange1 else None
    max_orange_area1 = cv2.contourArea(max_orange_contour1) if is_valid_contour(max_orange_contour1) else 0
    max_blue_contour2 = sortContourShapes(contours_blue2)[0] if contours_blue2 else None
    max_blue_area2 = cv2.contourArea(max_blue_contour2) if is_valid_contour(max_blue_contour2) else 0
    max_orange_contour2 = sortContourShapes(contours_orange2)[0] if contours_orange2 else None
    max_orange_area2 = cv2.contourArea(max_orange_contour2) if is_valid_contour(max_orange_contour2) else 0

    max_blue_area = max_blue_area2+max_blue_area1
    max_orange_area = max_orange_area2+max_orange_area1
    right_area = right_area_bot + right_area_top
    left_area = left_area_bot + left_area_top

    # end if 3 laps finished
    if max_turns <= turn_count:
        print("READY TO END")
        actions_to_straight += 1
        if actions_to_straight >= 25:  # finish turn and continue to go forwards for certain amount to enter straight section
            board.pwm_servo_set_position(0.1, [[1, 1500]])
            board.pwm_servo_set_position(0.1, [[2, 1500]])
            break


    # draw debug information
    if debug:
        drawContour(max_orange_contour1, ROI_LINE1, frame, color=(0, 165, 255), thickness=2)

        # Draw max blue contour (blue)
        drawContour(max_blue_contour1, ROI_LINE1, frame, color=(255, 0, 0), thickness=2)    

        drawContour(max_orange_contour2, ROI_LINE2, frame, color=(0, 165, 255), thickness=2)

        # Draw max blue contour (blue)
        drawContour(max_blue_contour2, ROI_LINE2, frame, color=(255, 0, 0), thickness=2)
    
        # Draw max black contours (magenta for visibility)
        drawContour(max_left_top_contour, ROI_LEFT_TOP, frame, color=(255, 0, 255), thickness=2)

        drawContour(max_right_top_contour, ROI_RIGHT_TOP, frame, color=(255, 0, 255), thickness=2)

        drawContour(max_left_bot_contour, ROI_LEFT_BOT, frame, color=(255, 0, 255), thickness=2)

        drawContour(max_right_bot_contour, ROI_RIGHT_BOT, frame, color=(255, 0, 255), thickness=2)

        # Draw all ROIs with logical color coding
        
        drawRect(ROI_LEFT_TOP, frame, color=(255, 0, 0), thickness=2)        # Blue - top left
        drawRect(ROI_RIGHT_TOP, frame, color=(0, 0, 255), thickness=2)       # Red - top right
        drawRect(ROI_LEFT_BOT, frame, color=(0, 255, 0), thickness=2)       # Green - bottom left
        drawRect(ROI_RIGHT_BOT, frame, color=(255, 255, 0), thickness=2)     # Cyan - bottom right
        drawRect(ROI_LINE1, frame, color=(0, 255, 255), thickness=2)     # Yellow - center/orientation line
        drawRect(ROI_LINE2, frame, color=(0, 255, 255), thickness=2)     # Yellow - center/orientation line
        cv2.imshow("Region of Interest", frame)
        cv2.waitKey(1)

    # find difference
    curr_diff = right_area-left_area

    if right_area > 1250 and left_area > 1250:
        angle = int((curr_diff * PGLOW + (curr_diff-last_diff) * PDLOW)) # change PID to lower for narrow configuration
    elif right_area <= WALL_THRESH:
        angle = -20 # turn hard right if wall on right lost
    elif left_area <= WALL_THRESH:
        angle = 15 # turn hard left if wall on left lost, angles are different due to servo imbalance
    else:
        angle = int((curr_diff * PG + (curr_diff-last_diff) * PD)) # otherwise in straight, so use PID control


    if turning:
        print("turning", turn_count)
        # conditions to exit turn status
        if track_dir == 1:
            if max_blue_area >= LINE_THRESH:
                turning = False
                turn_count += 1

        if track_dir == -1:
            if max_orange_area >= LINE_THRESH:
                turning = False
                turn_count += 1

    elif track_dir == 0:
        # find track direction using line contours, switch rois to side
        if max_orange_area >= LINE_THRESH:
            track_dir = 1
            ROI_LINE1 = [0,0,0,0]
            ROI_LINE2 = [40,420,115,445]
        elif max_blue_area >= LINE_THRESH:
            track_dir = -1
            ROI_LINE2 = [0,0,0,0]
            ROI_LINE1 = [525,420,600,445]

    # initiate turn sequences if line contour thresh passed
    elif track_dir == 1:
        if max_orange_area >= LINE_THRESH:
            turning = True

    elif track_dir == -1:
        if max_blue_area >= LINE_THRESH:
            turning = True
            
    


    # set servo and dc values on hardware
    servo = angle
    dc = speed
    print(left_area,right_area,servo)
    board.pwm_servo_set_position(0.1, [[1, pwm(clamp_to_max(angle+MID_SERVO))]])
    board.pwm_servo_set_position(0.1, [[2, dc]])
    time.sleep(0.1)

    last_diff = curr_diff
