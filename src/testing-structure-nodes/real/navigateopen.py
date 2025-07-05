import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
from std_msgs.msg import Float32


PD = 0.2
PG = 0.0035

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

        self.subscription_cam = self.create_subscription(Twist, "camera", self.cam_call,
10)
        self.subscription_imu = self.create_subscription(Float32, "imu", self.imu_call,
10)

        self.client = self.create_client(Twist, 'send_command')

        self.mode = None
        self.run()


    def send_command(self, servo, dc):
        request = Twist()
        request.servo = servo
        request.dc = dc
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
        
    def cam_call(self, msg):
        self.left_area = msg.wall_area_left
        self.right_area = msg.wall_area_right
        self.max_orange_area = msg.line_orange_area
        self.max_blue_area = msg.line_blue_area

    def imu_call(self,msg):
        self.current_angle = msg.data

    def run(self):
        while True:
            self.curr_diff = self.left_area - self.right_area

            angle = int(self.curr_diff * PG + (self.curr_diff-self.last_diff) * PD)
            self.send_command(angle,self.speed)

            self.last_diff = self.curr



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
