import rclpy
import cv2
import numpy as np
from picamera2 import Picamera2
import threading
import ros_robot_controller_sdk as rcc
from helper import *
from CONSTS import *
#from exit_parking import unpark
import time
board = rcc.Board()

#red is right
#green is left

debug = False

speed = 1608
last_pil_diff = 0
last_diff = 0
turning = False
turn_dir = 0
track_dir = 0
curr_diff = 0
error = 0
angle = 0
turn_count = 0
time_after_turn = 0
target = 0

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



while True:
    frame = picam2.capture_array()
    frame = cv2.resize(frame, (640, 480))
    
    if turn_count >= 12:
        board.pwm_servo_set_position(0.1, [[1, pwm(MID_SERVO)]])
        board.pwm_servo_set_position(0.1, [[2, 1500]])
        time.sleep(0.1)
        break


    img_lab = cv2.cvtColor(frame, cv2.COLOR_BGR2Lab)
    #img_lab = cv2.GaussianBlur(img_lab, (7, 7), 0)
    
    

    left_contours_top = findContours(img_lab, rBlack, ROI_LEFT_TOP)
    right_contours_top = findContours(img_lab, rBlack, ROI_RIGHT_TOP)
    left_contours_bot = findContours(img_lab, rBlack, ROI_LEFT_BOT)
    right_contours_bot = findContours(img_lab, rBlack, ROI_RIGHT_BOT)

    contours_blue1 = findContours(img_lab, rBlue, ROI_LINE1)

    contours_orange1 = findContours(img_lab, rOrange, ROI_LINE1)
    contours_blue2 = findContours(img_lab, rBlue, ROI_LINE2)

    contours_orange2 = findContours(img_lab, rOrange, ROI_LINE2)

    
    contours_red = findContours(img_lab, rRed, ROI_PILLAR)

    contours_green = findContours(img_lab, rGreen, ROI_PILLAR)
    
    for name, contours in [
    ("Left Top", left_contours_top),
    ("Right Top", right_contours_top),
    ("Left Bot", left_contours_bot),
    ("Right Bot", right_contours_bot),
    ("Blue1", contours_blue1),
    ("Orange1", contours_orange1),
    ("Blue2", contours_blue2),
    ("Orange2", contours_orange2),
    ("Red", contours_red),
    ("Green", contours_green),
]:
        if contours:
            areas = [cv2.contourArea(c) for c in contours]
            #print(f"{name} -> Count: {len(contours)}, Areas: {areas}")
            if "Blue" in name or "Orange" in name:
                #print("TURN INITISIUDIS DISDFHKJSFKJSDFKJDHSJDHJKDSBKJBDSBJFSFKJSFKJSKJBFJBSJKSJKFBKJSFBKSFBKJSFJKFBKFSKBFKJSBFKABSKSBKJSBJFBJSKJFKSBKJBSDKSBSFF")
                pass
        else:
            #print(f"{name} -> None")
            pass


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


    max_blue_contour1 = sortContourShapes(contours_blue1)[0] if contours_blue1 else None
    max_blue_area1 = cv2.contourArea(max_blue_contour1) if is_valid_contour(max_blue_contour1) else 0
    max_orange_contour1 = sortContourShapes(contours_orange1)[0] if contours_orange1 else None
    max_orange_area1 = cv2.contourArea(max_orange_contour1) if is_valid_contour(max_orange_contour1) else 0
    max_blue_contour2 = sortContourShapes(
        angle=40
    elif angle<-40:
        angle=-40

    servo = angle
    print(angle,speed)
    dc = speed
    board.pwm_servo_set_position(0.1, [[1, pwm(angle+MID_SERVO)]])
    board.pwm_servo_set_position(0.1, [[2, dc]])

    last_diff = curr_diff
    last_pill_diff= error

