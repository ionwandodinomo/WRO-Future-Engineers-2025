import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from std_msgs.msg import Bool
from picamera2 import Picamera2
from std_msgs.msg import Int32MultiArray

LOWER_BLACK_THRESHOLD = np.array([0, 0, 0])
UPPER_BLACK_THRESHOLD = np.array([180, 255, 62])

LOWER_BLUE = np.array([107, 88, 0])
UPPER_BLUE = np.array([121, 255, 211])

LOWER_ORANGE1 = np.array([0, 101, 169])
UPPER_ORANGE1 = np.array([27, 219, 255])
LOWER_ORANGE2 = np.array([0, 0, 0])
UPPER_ORANGE2 = np.array([0, 0, 0])



ROI_LEFT_TOP = [0, 245, 100, 285]        
ROI_RIGHT_TOP = [540, 250, 640, 290]
ROI_LEFT_BOT = [0, 285, 40, 300]
ROI_RIGHT_BOT = [600, 290, 640, 305]

ROI_LINE = [277,300,75,25]

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

    
class CameraNode(Node):
    
    def __init__(self):
        super().__init__('camera_node')
        self.picam2 = Picamera2()
        self.picam2.preview_configuration.main.size = (640,480)
        self.picam2.preview_configuration.main.format = "RGB888"
        print(self.picam2.preview_configuration.controls.FrameRate)
        self.picam2.preview_configuration.controls.FrameRate = 25
        self.picam2.set_controls({"Brightness": 0.05})
        print(self.picam2.preview_configuration.controls.FrameRate)
        self.picam2.preview_configuration.align()
        self.picam2.configure("preview")
        self.picam2.start()

        self.publisher_ = self.create_publisher(Int32MultiArray, 'camera', 10)

        self.timer = self.create_timer(0.1, self.process_frame)



    def process_frame(self):
        
        frame = self.picam2.capture_array()
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
            b_mask[ROI_LINE[1] : ROI_LINE[3], ROI_LINE[0] : ROI_LINE[2]],
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )[-2]

        contours_orange = cv2.findContours(
            o_mask[ROI_LINE[1] : ROI_LINE[3], ROI_LINE[0] : ROI_LINE[2]],
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
            if max_orange_contour is not None:
                max_orange_contour[:, :, 0] += ROI_LINE[0]
                max_orange_contour[:, :, 1] += ROI_LINE[1]
                cv2.drawContours(frame, [max_orange_contour], -1, (0, 165, 255), 2)

            if max_blue_contour is not None:
                max_blue_contour[:, :, 0] += ROI_LINE[0]
                max_blue_contour[:, :, 1] += ROI_LINE[1]
                cv2.drawContours(frame, [max_blue_contour], -1, (255, 0, 0), 2)
            
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
                        (ROI_LINE[0] + ROI_LINE[2], ROI_LINE[1] + ROI_LINE[3]), 
                        (0, 255, 255), 2)     # Yellow - center/orientation line

            cv2.imshow("Region of Interest", frame)
            cv2.waitKey(1)


        msg = Int32MultiArray()
        msg.data = [int(left_area),int(right_area), int(max_orange_area),int(max_blue_area)]

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    rclpy.spin(camera_node)
    camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


