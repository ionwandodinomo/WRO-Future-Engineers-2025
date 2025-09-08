import cv2
import numpy as np
from picamera2 import Picamera2

LOWER_BLACK_THRESHOLD = np.array([0, 0, 0])
UPPER_BLACK_THRESHOLD = np.array([180, 255, 100])

LOWER_BLUE = np.array([91, 115, 103])
UPPER_BLUE = np.array([132, 255, 255])

LOWER_ORANGE1 = np.array([0, 101, 173])
UPPER_ORANGE1 = np.array([27, 255, 255])
LOWER_ORANGE2 = np.array([0, 0, 0])
UPPER_ORANGE2 = np.array([0, 0, 0])

#ROI_LEFT_TOP = [0, 220, 100, 270]        
#ROI_RIGHT_TOP = [540, 240, 640, 290]
ROI_LEFT_TOP = [0, 220, 175, 270]        
ROI_RIGHT_TOP = [465, 240, 640, 290]
#ROI_LEFT_BOT = [0, 400, 40, 425]
#ROI_RIGHT_BOT = [600, 420, 640, 445]
ROI_LEFT_BOT = [0, 270, 115, 295]
ROI_RIGHT_BOT = [525, 290, 640, 315]


ROI_LINE1 = [525,400,600,425]
ROI_LINE2 = [40,400,115,425]


debug = True



def findMaxContourShape(contours):
    max_area = 0
    max_contour = None
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > max_area:
            max_area = area
            max_contour = cnt
    return max_contour, max_area


picam2 = Picamera2()
picam2.preview_configuration.main.size = (640,480)
picam2.preview_configuration.main.format = "RGB888"
print(picam2.preview_configuration.controls.FrameRate)
picam2.preview_configuration.controls.FrameRate = 50
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
        b_mask[ROI_LINE1[1] : ROI_LINE1[3], ROI_LINE1[0] : ROI_LINE1[2]],
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )[-2]

    contours_orange = cv2.findContours(
        o_mask[ROI_LINE1[1] : ROI_LINE1[3], ROI_LINE1[0] : ROI_LINE1[2]],
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )[-2]

    contours_blue = cv2.findContours(
        b_mask[ROI_LINE2[1] : ROI_LINE2[3], ROI_LINE2[0] : ROI_LINE2[2]],
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )[-2]

    contours_orange = cv2.findContours(
        o_mask[ROI_LINE2[1] : ROI_LINE2[3], ROI_LINE2[0] : ROI_LINE2[2]],
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )[-2]
    

    max_left_top_contour, left_area_top = findMaxContourShape(left_contours_top)
    max_right_top_contour, right_area_top = findMaxContourShape(right_contours_top)
    max_left_bot_contour, left_area_bot = findMaxContourShape(left_contours_bot)
    max_right_bot_contour, right_area_bot = findMaxContourShape(right_contours_bot)

    max_blue_contour, max_blue_area = findMaxContourShape(contours_blue)
    max_orange_contour, max_orange_area = findMaxContourShape(contours_orange)


    right_area = right_area_bot + right_area_top
    left_area = left_area_bot + left_area_top

    

    if debug:
        # Draw max orange contour (orange)
        if max_orange_contour is not None:
            max_orange_contour[:, :, 0] += ROI_LINE1[0]
            max_orange_contour[:, :, 1] += ROI_LINE1[1]
            cv2.drawContours(frame, [max_orange_contour], -1, (0, 165, 255), 2)

        # Draw max blue contour (blue)
        if max_blue_contour is not None:
            max_blue_contour[:, :, 0] += ROI_LINE1[0]
            max_blue_contour[:, :, 1] += ROI_LINE1[1]
            cv2.drawContours(frame, [max_blue_contour], -1, (255, 0, 0), 2)

        if max_orange_contour is not None:
            max_orange_contour[:, :, 0] += ROI_LINE2[0]
            max_orange_contour[:, :, 1] += ROI_LINE2[1]
            cv2.drawContours(frame, [max_orange_contour], -1, (0, 165, 255), 2)

        # Draw max blue contour (blue)
        if max_blue_contour is not None:
            max_blue_contour[:, :, 0] += ROI_LINE2[0]
            max_blue_contour[:, :, 1] += ROI_LINE2[1]
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
                      (ROI_LINE1[0], ROI_LINE1[1]), 
                      (ROI_LINE1[2], ROI_LINE1[3]), 
                      (0, 255, 255), 2)     # Yellow - center/orientation line
        
        cv2.rectangle(frame, 
                      (ROI_LINE2[0], ROI_LINE2[1]), 
                      (ROI_LINE2[2], ROI_LINE2[3]), 
                      (0, 255, 255), 2)     # Yellow - center/orientation line

        cv2.imshow("Region of Interest", frame)
        cv2.waitKey(1)

