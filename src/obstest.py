import rclpy
import cv2
import numpy as np
from picamera2 import Picamera2
import ros_robot_controller_sdk as rcc
import time
board = rcc.Board()

#red is right
#green is left

def pwm(degree):  
    pw = round(degree * 11.1 + 1500)
    return max(0, min(65535, pw))

LOWER_BLACK_THRESHOLD = np.array([0, 0, 0])
UPPER_BLACK_THRESHOLD = np.array([180, 255, 83])

LOWER_BLUE = np.array([91, 115, 103])
UPPER_BLUE = np.array([132, 255, 255])

LOWER_ORANGE1 = np.array([0, 101, 173])
UPPER_ORANGE1 = np.array([27, 255, 255])
LOWER_ORANGE2 = np.array([0, 0, 0])
UPPER_ORANGE2 = np.array([0, 0, 0])

LOWER_RED_THRESHOLD1 = np.array([0, 120, 138])
UPPER_RED_THRESHOLD1 = np.array([8, 255, 255])
LOWER_RED_THRESHOLD2 = np.array([163, 120, 138]) 
UPPER_RED_THRESHOLD2 = np.array([180, 255, 255]) 

LOWER_GREEN_THRESHOLD = np.array([71, 82, 75])
UPPER_GREEN_THRESHOLD = np.array([100, 255, 178])

LOWER_MAGENTA_THRESHOLD1 = np.array([0, 0, 0])
UPPER_MAGENTA_THRESHOLD1 = np.array([0, 0, 0])
LOWER_MAGENTA_THRESHOLD2 = np.array([137, 168, 173])
UPPER_MAGENTA_THRESHOLD2 = np.array([162, 255, 255])


ROI_LEFT_TOP = [0, 220, 100, 270]        
ROI_RIGHT_TOP = [540, 220, 640, 270]
ROI_LEFT_BOT = [0, 270, 40, 295]
ROI_RIGHT_BOT = [600, 270, 640, 295]
# ROI_LEFT_TOP = [0, 220, 225, 270]        
# ROI_RIGHT_TOP = [415, 240, 640, 290]
# ROI_LEFT_BOT = [0, 270, 115, 295]
# ROI_RIGHT_BOT = [525, 290, 640, 315]

ROI_LINE = [277,300,352,325]
ROI_PILLAR = [0,140,640,400]
RED_TARGET = 110
GREEN_TARGET = 530

# PD = 0.00003
# PG = 0.00002
# PILLAR_PD = 0.00003
# PILLAR_PG = 0.00002
PD = 0.0000002
PG = 0.000003
PDLOW = 0.00001
PGLOW = 0.0015
PILLAR_PD = 0.005
PILLAR_PG = 0.0035
LINE_THRESH = 120
WALL_THRESH = 20
MAX_TURN_DEGREE = 40
PILLAR_THRESH = 150
MID_SERVO = -6
speed = 1620
last_diff = 0
turning = False
turn_dir = 0
track_dir = 0
curr_diff = 0


debug = True

def findMaxContourShape(contours):
    if contours is None:
        raise Exception
    max_area = 0
    max_contour = None
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > max_area:
            max_area = area
            max_contour = cnt
    return max_contour, max_area

def is_valid_contour(c):
    return c is not None and isinstance(c, (list, tuple, np.ndarray)) and len(c) > 0

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


    img_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    img_thresh = cv2.inRange(img_hsv, LOWER_BLACK_THRESHOLD, UPPER_BLACK_THRESHOLD)
    b_mask = cv2.inRange(img_hsv, LOWER_BLUE, UPPER_BLUE)
    o_mask = cv2.bitwise_or(
        cv2.inRange(img_hsv, LOWER_ORANGE1, UPPER_ORANGE1),
        cv2.inRange(img_hsv, LOWER_ORANGE2, UPPER_ORANGE2),
    )
    r_mask = cv2.bitwise_or(
            cv2.inRange(img_hsv, LOWER_RED_THRESHOLD1, UPPER_RED_THRESHOLD1),
            cv2.inRange(img_hsv, LOWER_RED_THRESHOLD2, UPPER_RED_THRESHOLD2),
        )
    g_mask = cv2.inRange(img_hsv, LOWER_GREEN_THRESHOLD, UPPER_GREEN_THRESHOLD)

    left_contours_top, hierarchy = cv2.findContours(
        img_thresh[
            ROI_LEFT_TOP[1] : ROI_LEFT_TOP[3], ROI_LEFT_TOP[0] : ROI_LEFT_TOP[2]
        ],
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_NONE,
    )
    right_contours_top, hierarchy = cv2.findContours(
        img_thresh[
            ROI_RIGHT_TOP[1] : ROI_RIGHT_TOP[3], ROI_RIGHT_TOP[0] : ROI_RIGHT_TOP[2]
        ],
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_NONE,
    )
    left_contours_bot, hierarchy = cv2.findContours(
        img_thresh[
            ROI_LEFT_BOT[1] : ROI_LEFT_BOT[3], ROI_LEFT_BOT[0] : ROI_LEFT_BOT[2]
        ],
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_NONE,
    )
    right_contours_bot, hierarchy = cv2.findContours(
        img_thresh[
            ROI_RIGHT_BOT[1] : ROI_RIGHT_BOT[3], ROI_RIGHT_BOT[0] : ROI_RIGHT_BOT[2]
        ],
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_NONE,
    )

    contours_blue = cv2.findContours(
        b_mask[ROI_LINE[1] : ROI_LINE[3], ROI_LINE[0] : ROI_LINE[2]],
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )[-2]


    contours_orange = cv2.findContours(
        o_mask[ROI_LINE[1] : ROI_LINE[3], ROI_LINE[0] : ROI_LINE[2]],
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )[-2]
    contours_red = cv2.findContours(
            r_mask[ROI_PILLAR[1] : ROI_PILLAR[3], ROI_PILLAR[0] : ROI_PILLAR[2]],
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )[-2]

    contours_green = cv2.findContours(
        g_mask[ROI_PILLAR[1] : ROI_PILLAR[3], ROI_PILLAR[0] : ROI_PILLAR[2]],
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )[-2]

    max_left_top_contour, left_area_top = findMaxContourShape(left_contours_top)
    max_right_top_contour, right_area_top = findMaxContourShape(right_contours_top)
    max_left_bot_contour, left_area_bot = findMaxContourShape(left_contours_bot)
    max_right_bot_contour, right_area_bot = findMaxContourShape(right_contours_bot)

    max_blue_contour, max_blue_area = findMaxContourShape(contours_blue)
    max_orange_contour, max_orange_area = findMaxContourShape(contours_orange)
    max_red_contour, max_red_area = findMaxContourShape(contours_red)
    max_green_contour, max_green_area = findMaxContourShape(contours_green)
    right_area = right_area_bot + right_area_top
    left_area = left_area_bot + left_area_top
    if debug:
        # Draw max orange contour (orange)
        if max_orange_contour is not None:
            max_orange_contour[:, :, 0] += ROI_LINE[0]
            max_orange_contour[:, :, 1] += ROI_LINE[1]
            cv2.drawContours(frame, [max_orange_contour], -1, (0, 165, 255), 2)

        # Draw max blue contour (blue)
        if max_blue_contour is not None:
            max_blue_contour[:, :, 0] += ROI_LINE[0]
            max_blue_contour[:, :, 1] += ROI_LINE[1]
            cv2.drawContours(frame, [max_blue_contour], -1, (255, 0, 0), 2)

        # Draw max black contours (magenta for visibility)
        if max_left_top_contour is not None:
            max_left_top_contour[:, :, 0] += ROI_LEFT_TOP[0]
            max_left_top_contour[:, :, 1] += ROI_LEFT_TOP[1]
            cv2.drawContours(frame, [max_left_top_contour], -1, (255, 0, 255), 2)

        if max_right_top_contour is not None:
            max_right_top_contour[:, :, 0] += ROI_RIGHT_TOP[0]
            max_right_top_contour[:, :, 1] += ROI_RIGHT_TOP[1]
            cv2.drawContours(frame, [max_right_top_contour], -1, (255, 0, 255), 2)

        if max_left_bot_contour is not None:
            max_left_bot_contour[:, :, 0] += ROI_LEFT_BOT[0]
            max_left_bot_contour[:, :, 1] += ROI_LEFT_BOT[1]
            cv2.drawContours(frame, [max_left_bot_contour], -1, (255, 0, 255), 2)

        if max_right_bot_contour is not None:
            max_right_bot_contour[:, :, 0] += ROI_RIGHT_BOT[0]
            max_right_bot_contour[:, :, 1] += ROI_RIGHT_BOT[1]
            cv2.drawContours(frame, [max_right_bot_contour], -1, (255, 0, 255), 2)

        if max_green_contour is not None:
            max_green_contour[:, :, 0] += ROI_PILLAR[0]
            max_green_contour[:, :, 1] += ROI_PILLAR[1]
            cv2.drawContours(frame, [max_green_contour], -1, (0, 255, 0), 2)

        if max_red_contour is not None:
            max_red_contour[:, :, 0] += ROI_PILLAR[0]
            max_red_contour[:, :, 1] += ROI_PILLAR[1]
            cv2.drawContours(frame, [max_red_contour], -1, (0, 0, 255), 2)

        # Draw all ROIs with logical color coding
        cv2.rectangle(frame, 
                    (ROI_LEFT_TOP[0], ROI_LEFT_TOP[1]), 
                    (ROI_LEFT_TOP[2], ROI_LEFT_TOP[3]), 
                    (255, 0, 0), 2)        # Blue - top left

        cv2.rectangle(frame, 
                    (ROI_RIGHT_TOP[0], ROI_RIGHT_TOP[1]), 
                    (ROI_RIGHT_TOP[2], ROI_RIGHT_TOP[3]), 
                    (0, 0, 255), 2)       # Red - top right

        cv2.rectangle(frame, 
                    (ROI_LEFT_BOT[0], ROI_LEFT_BOT[1]), 
                    (ROI_LEFT_BOT[2], ROI_LEFT_BOT[3]), 
                    (0, 255, 0), 2)       # Green - bottom left

        cv2.rectangle(frame, 
                    (ROI_RIGHT_BOT[0], ROI_RIGHT_BOT[1]), 
                    (ROI_RIGHT_BOT[2], ROI_RIGHT_BOT[3]), 
                    (255, 255, 0), 2)     # Cyan - bottom right

        cv2.rectangle(frame, 
                    (ROI_LINE[0], ROI_LINE[1]), 
                    (ROI_LINE[2], ROI_LINE[3]), 
                    (0, 255, 255), 2)     # Yellow - center/orientation line
        
        cv2.rectangle(frame, 
                    (ROI_PILLAR[0], ROI_PILLAR[1]), 
                    (ROI_PILLAR[0] + ROI_PILLAR[2], ROI_PILLAR[1] + ROI_PILLAR[3]), 
                    (0, 255, 255), 2)

    if is_valid_contour(max_red_contour) and is_valid_contour(max_green_contour):
        if PILLAR_THRESH < cv2.contourArea(max_red_contour) > cv2.contourArea(max_green_contour):
            selected_contour = max_red_contour
            target = RED_TARGET
            sign = 1
            cv2.line(frame, (RED_TARGET, 0), (RED_TARGET, 480), (0, 0, 255), 2)
        elif PILLAR_THRESH < cv2.contourArea(max_green_contour):
            selected_contour = max_green_contour
            target = GREEN_TARGET
            sign = -1
            cv2.line(frame, (GREEN_TARGET, 0), (GREEN_TARGET, 480), (0, 255, 0), 2)
        else:
            selected_contour = None

    elif is_valid_contour(max_red_contour) and PILLAR_THRESH < cv2.contourArea(max_red_contour):
        selected_contour = max_red_contour
        target = RED_TARGET
        sign = 1
        cv2.line(frame, (RED_TARGET, 0), (RED_TARGET, 480), (0, 0, 255), 2)

    elif is_valid_contour(max_green_contour) and PILLAR_THRESH < cv2.contourArea(max_green_contour):
        selected_contour = max_green_contour
        target = GREEN_TARGET
        sign = -1
        cv2.line(frame, (GREEN_TARGET, 0), (GREEN_TARGET, 480), (0, 255, 0), 2)

    else:
        selected_contour = None


    if selected_contour is not None:
        x, y, w, h = cv2.boundingRect(selected_contour)
        cX = x + w // 2
        cY = y + h // 2

        # Normalize y-distance
        pillar_roi_bottom = ROI_PILLAR[1] + ROI_PILLAR[3]
        pillar_roi_top = ROI_PILLAR[1]
        normalized_y = (cY - pillar_roi_top) / (pillar_roi_bottom - pillar_roi_top)
        normalized_y = np.clip(normalized_y, 0.0, 1.0)

        # Dynamic gain
        dynamic_gain = PILLAR_PG + (1 - normalized_y) * 0.0005

        # Calculate error with correct sign
#         if sign == 1:
#             error = cX - target
#         else:
#             error = target - cX
        
        error = cX-target
        
        angle = int((-MAX_TURN_DEGREE * (error * dynamic_gain + ((error - last_diff) * PILLAR_PD))))#*(normalized_y+1))#*(normalized_y/2+0.5))
        
#         if normalized_y > 0.545:
#             curr_diff = right_area - left_area
#             angle = int(-MAX_TURN_DEGREE * (-curr_diff * PILLAR_PG + (-curr_diff - last_diff) * PILLAR_PD))
#             angle=0

#             if right_area > 1250 and left_area > 1250:
#                 angle = int((curr_diff * PGLOW + (curr_diff-last_diff) * PDLOW))
#             else:
#             curr_diff = right_area - left_area
#             angle = int((curr_diff * PG + (curr_diff-last_diff) * PD))

#         if normalized_y < 0.15:
#             angle=angle/2
        if normalized_y > 0.3:
            RED_TARGET = 50
            GREEN_TARGET = 590
        else:
            RED_TARGET = 110
            GREEN_TARGET = 530


        print (error, dynamic_gain, error, last_diff, cX, target, normalized_y)
#         angle = int(-MAX_TURN_DEGREE * (error * dynamic_gain * PILLAR_PD))


        # Optional: reduce speed when close
        if normalized_y < 0.3:
            speed = 1600
        else:
            speed = 1600

        # Debug draw center
        cv2.circle(frame, (cX, cY), 5, (255, 255, 255), -1)
    else:
        # Fall back to black area balance logic
        curr_diff = right_area - left_area
        angle = int(-MAX_TURN_DEGREE * (-curr_diff * PILLAR_PG + (-curr_diff - last_diff) * PILLAR_PD))
#         if right_area > 1250 and left_area > 1250:
#             angle = int((curr_diff * PGLOW + (curr_diff-last_diff) * PDLOW))
#         else:
#         angle = int((curr_diff * PG + (curr_diff-last_diff) * PD))
    cv2.imshow("Region of Interest", frame)
    cv2.waitKey(1)
    
    print(angle)
    """if turning:
        print("turning")
        if track_dir == 1:
            if max_blue_area >= LINE_THRESH:
                angle = 0
                turning = False
            else:
                angle = -MAX_TURN_DEGREE

        if track_dir == 0:
            if max_orange_area >= LINE_THRESH:
                angle = 0
                turning = False
            else:
                angle = MAX_TURN_DEGREE

    if track_dir == 0:
        if max_orange_area >= LINE_THRESH:
            track_dir = 1
            turning = True
        elif max_blue_area >= LINE_THRESH:
            track_dir = -1
            turning = True

    elif track_dir == 1:
        if max_orange_area >= LINE_THRESH:
            turning = True

        elif max_blue_area >= LINE_THRESH:
            angle = 0

    elif track_dir == -1:
        if max_orange_area >= LINE_THRESH:
            angle = 0
        elif max_blue_area >= LINE_THRESH:
            turning = True
    """
    
    if angle>40:
        angle=40
    elif angle<-40:
        angle=-40
    
    servo = angle
    print(angle,speed)
    dc = 1600
    board.pwm_servo_set_position(0.1, [[1, pwm(angle+MID_SERVO)]])
    board.pwm_servo_set_position(0.1, [[2, dc]])
    time.sleep(0.1)

    last_diff = curr_diff



    

    

