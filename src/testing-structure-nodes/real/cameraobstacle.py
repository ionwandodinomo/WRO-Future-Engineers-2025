import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from std_msgs.msg import Bool
from picamera2 import Picamera2
from geometry_msgs.msg import Twist


LINE_THRESHOLD = 120 

PILLAR_SIZE = 320

DC_SPEED = 1342
MID_SERVO = 82
MAX_TURN_DEGREE = 42

LOWER_RED_THRESHOLD1 = np.array([0, 154, 70])
UPPER_RED_THRESHOLD1 = np.array([4, 255, 255])
LOWER_RED_THRESHOLD2 = np.array([174, 180, 70])
UPPER_RED_THRESHOLD2 = np.array([180, 255, 255])

LOWER_GREEN_THRESHOLD = np.array([65, 85, 40])
UPPER_GREEN_THRESHOLD = np.array([105, 255, 185])

LOWER_MAGENTA_THRESHOLD1 = np.array([0, 100, 50])
UPPER_MAGENTA_THRESHOLD1 = np.array([0, 215, 255])

LOWER_MAGENTA_THRESHOLD2 = np.array([150, 150, 70])
UPPER_MAGENTA_THRESHOLD2 = np.array([172, 255, 255])

LOWER_ORANGE1 = np.array([180, 100, 100])
UPPER_ORANGE1 = np.array([180, 255, 255])
LOWER_ORANGE2 = np.array([0, 80, 80])
UPPER_ORANGE2 = np.array([20, 255, 255])

LOWER_BLUE = np.array([101, 20, 30])
UPPER_BLUE = np.array([125, 255, 255])

LOWER_BLACK_THRESHOLD = np.array([0, 0, 0])
UPPER_BLACK_THRESHOLD = np.array([180, 255, 80])

ROI_LEFT_BOT = [0, 300, 100, 340]
ROI_RIGHT_BOT = [540, 300, 640, 340]
ROI_LEFT_TOP = [0, 285, 40, 300]
ROI_RIGHT_TOP = [600, 285, 640, 300]

ROI_LINE = [0,0,50,50]


debug = True
started = False

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
        

        self.publisher_ = self.create_publisher(Twist, 'camera', 10)

        """
        self.subscription = self.create_subscription(
            Bool,
            "/startopen",
            self.change_state,
            10)"""


        self.timer = self.create_timer(0.1, self.process_frame)



    def process_frame(self):
        if not started:
            return
        
        frame = self.picam2.capture_array()
        frame = cv2.resize(frame, (640, 480))
        
        if debug:
            cv2.rectangle(frame, (ROI_LEFT_TOP[0], ROI_LEFT_TOP[1]),
                  (ROI_LEFT_TOP[0] + ROI_LEFT_TOP[2], ROI_LEFT_TOP[1] + ROI_LEFT_TOP[3]), (255, 0, 0), 2)
            cv2.rectangle(frame, (ROI_RIGHT_TOP[0], ROI_RIGHT_TOP[1]),
                  (ROI_RIGHT_TOP[0] + ROI_RIGHT_TOP[2], ROI_RIGHT_TOP[1] + ROI_RIGHT_TOP[3]), (0, 0, 255), 2)
        


            cv2.imshow("Region of Interest", frame)


        img_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        img_thresh = cv2.inRange(img_hsv, LOWER_BLACK_THRESHOLD, UPPER_BLACK_THRESHOLD)


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

        left_area_top = 0
        left_area_bot = 0

        right_area_top = 0
        right_area_bot = 0

        for i in range(len(left_contours_top)):
            cnt = left_contours_top[i]
            area = cv2.contourArea(cnt)
            left_area_top = max(area, left_area_top)
        for i in range(len(left_contours_bot)):
            cnt = left_contours_bot[i]
            area = cv2.contourArea(cnt)
            left_area_bot = max(area, left_area_bot)
        for i in range(len(right_contours_top)):
            cnt = right_contours_top[i]
            area = cv2.contourArea(cnt)
            right_area_top = max(area, right_area_top)
        for i in range(len(right_contours_bot)):
            cnt = right_contours_bot[i]
            area = cv2.contourArea(cnt)
            right_area_bot = max(area, right_area_bot)

        right_area = right_area_bot + right_area_top
        left_area = left_area_bot + left_area_top

        b_mask = cv2.inRange(img_hsv, LOWER_BLUE, UPPER_BLUE)
        contours_blue = cv2.findContours(
            b_mask[ROI_LINE[1] : ROI_LINE[3], ROI_LINE[0] : ROI_LINE[2]],
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )[-2]

 
        o_mask = cv2.bitwise_or(
            cv2.inRange(img_hsv, LOWER_ORANGE1, UPPER_ORANGE1),
            cv2.inRange(img_hsv, LOWER_ORANGE2, UPPER_ORANGE2),
        )

        contours_orange = cv2.findContours(
            o_mask[ROI_LINE[1] : ROI_LINE[3], ROI_LINE[0] : ROI_LINE[2]],
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )[-2]

        max_blue_area = 0
        max_orange_area = 0

        for i in range(len(contours_orange)):
            cnt = contours_orange[i]
            max_orange_area = max(cv2.contourArea(cnt), max_orange_area)
            cnt[:, :, 0] += ROI_LINE[0]  # x offset
            cnt[:, :, 1] += ROI_LINE[1]  # y offset
            if self.debug:
                cv2.drawContours(
                    frame, contours_orange, i, (255, 255, 0), 1
                )
        for i in range(len(contours_blue)):
            cnt = contours_blue[i]
            max_blue_area = max(cv2.contourArea(cnt), max_blue_area)
            cnt[:, :, 0] += ROI_LINE[0]  # x offset
            cnt[:, :, 1] += ROI_LINE[1]  # y offset
            
            if self.debug:
                cv2.drawContours(
                    frame, contours_blue, i, (255, 255, 0), 1
                )

        msg = Twist()
        msg.wall_area_left = left_area
        msg.wall_area_right = right_area
        msg.line_orange_area = max_orange_area
        msg.line_blue_area = max_blue_area

        self.publisher_.publish(msg)

    def change_state(self,msg:Bool):
        if msg.data == False:
            if self.started:
                cv2.destroyAllWindows()
                super.destroy_node()

        else:
            self.started = True
        

def main(args=None):
    # Initialize ROS2 node
    rclpy.init(args=args)
    # Create MinimalPublisher object
    camera_node = CameraNode()
    # Enter the event loop of ROS2 node
    rclpy.spin(camera_node)
    # Destroy node object
    camera_node.destroy_node()
    # Shut down ROS2 node
    rclpy.shutdown()
# If this script is the main program, main function is executed.
if __name__ == "__main__":
    main()


