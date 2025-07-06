import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import math
from std_msgs.msg import Float32
import time


PD = 0.2
PG = 0.0035
LINE_THRESH = 120
WALL_THRESH = 20
MAX_TURN_DEGREE = 50
TRACK_DIR = 0

servo = 0
dc = 0
LED1 = [0,0,0]
LED2 = [0,0,0]

turn_count = 0 
class NavigateNode(Node):
    def __init__(self):
        super().__init__('nagivate_node')
        self.left_area = 0
        self.right_area = 0
        self.max_orange_area = 0
        self.max_blue_area = 0
        self.speed = 1550
        self.curr_diff = 0
        self.last_diff = 0

        self.current_angle = 0

        self.subscription_cam = self.create_subscription(Int32MultiArray, "camera", self.cam_call,
10)
        self.subscription_imu = self.create_subscription(Float32, "imu", self.imu_call,
10)

        self.publisher = self.create_publisher(Int32MultiArray, 'send_command',10)
        self.timer = self.create_timer(0.01, self.send_command)

        self.LED = self.create_publisher(Int32MultiArray, 'LED_command', self.add_callback,10)
        self.timer = self.create_timer(0.1, self.send_LED)

        self.mode = None
        self.run()

    def send_LED(self):
        msg = Int32MultiArray()
        msg.data = [1,*LED1]
        msg.data = [2,*LED2]
        self.publisher_.publish(msg)
        return


    def send_command(self):
        global servo, dc
        request = Int32MultiArray()
        request.data = [servo,dc]
        self.publisher_.publish(request)
        self.get_logger().info(f'Angle: {servo}, speed: {dc}')
        
    def cam_call(self, msg):
        self.left_area = msg.data[0]
        self.right_area = msg.data[1]
        self.max_orange_area = msg.data[2]
        self.max_blue_area =msg.data[3]

    def imu_call(self,msg):
        self.current_angle = msg.data

    def run(self):
        global turn_count, servo,dc,angle
        LED1 = [255,255,0]
        time.sleep(1)
        while True:

            if turn_count == 12:
                break

            self.curr_diff = self.left_area - self.right_area

            angle = int(self.curr_diff * PG + (self.curr_diff-self.last_diff) * PD)

            if TRACK_DIR == 0:
                if self.max_orange_area >= LINE_THRESH:
                    TRACK_DIR = 1
                    turn_count += 1
                elif self.max_blue_area >= LINE_THRESH:
                    TRACK_DIR = -1

            elif TRACK_DIR == 1:
                if self.max_orange_area >= LINE_THRESH:
                    angle = MAX_TURN_DEGREE
                elif self.max_blue_area >= LINE_THRESH:
                    angle = 0
                    turn_count += 1
            elif TRACK_DIR == -1:
                if self.max_orange_area >= LINE_THRESH:
                    angle = 0
                    turn_count += 1
                elif self.max_blue_area >= LINE_THRESH:
                    angle = -MAX_TURN_DEGREE

            servo = angle
            dc = self.speed

            self.last_diff = self.curr

        servo = angle
        dc = self.speed
        time.sleep(1)
        return



def main(args=None):
    # Initialize ROS2 node
    rclpy.init(args=args)
    # Create MinimalSubscriber object
    navigate_node = NavigateNode()
    # Enter the event loop of ROS2 node
    rclpy.spin(navigate_node)
    # Destroy node object
    navigate_node.destroy_node()
    # Shut down ROS2 node
    rclpy.shutdown()
# If this script is the main program, the main function is executed.
if __name__ == "__main__":
    main()
