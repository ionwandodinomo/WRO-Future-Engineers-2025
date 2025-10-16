import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import math
from std_msgs.msg import Float32
import time


PD = 0.00003
PG = 0.00002
LINE_THRESH = 120
WALL_THRESH = 20
MAX_TURN_DEGREE = 50




class NavigateNode(Node):
    def __init__(self):
        super().__init__('nagivate_node')
        self.left_area = 0
        self.right_area = 0
        self.max_orange_area = 0
        self.max_blue_area = 0
        self.speed = 1620
        self.curr_diff = 0
        self.last_diff = 0
        self.track_dir = 0
        self.servo = 0
        self.dc = 0
        self.turn_count = 0 
        self.turning = False
        self.turning_start_angle = 0

        self.current_angle = 0

        self.subscription_cam = self.create_subscription(Int32MultiArray, "camera", self.cam_call,
10)
        self.subscription_imu = self.create_subscription(Float32, "imu", self.imu_call,
10)

        self.publisher = self.create_publisher(Int32MultiArray, 'send_command',10)
        self.timer_motors = self.create_timer(0.01, self.send_command)

        self.LED = self.create_publisher(Int32MultiArray, 'LED_command',10)
        self.timer_led = self.create_timer(0.1, self.send_LED)

        self.LED1 = [255,255,0]
        self.LED2 = [0,0,0]
        time.sleep(1)

        self.timer_logic = self.create_timer(0.01, self.run)

    def send_LED(self):
        msg = Int32MultiArray()
        msg.data = [1,*self.LED1]
        self.LED.publish(msg)
        msg.data = [2,*self.LED2]
        self.LED.publish(msg)
        return


    def send_command(self):
        request = Int32MultiArray()
        request.data = [self.servo,self.dc]
        self.publisher.publish(request)
        self.get_logger().info(f'Angle: {self.servo}, speed: {self.dc}')
        
    def cam_call(self, msg):
        self.left_area = msg.data[0]
        self.right_area = msg.data[1]
        self.max_orange_area = msg.data[2]
        self.max_blue_area = msg.data[3]

    def imu_call(self,msg):
        self.current_angle = msg.data
    
    def angleReached(self, delta):
        diff = (self.current_angle - self.turning_start_angle + 360) % 360
        self.get_logger().info(f"{diff} degrees away")
        return diff >= delta

    def run(self):
        global MAX_TURN_DEGREE,LINE_THRESH,WALL_THRESH,PD,PG
        if self.turn_count == 12:
            self.servo = 0
            self.dc = self.speed
            time.sleep(1)
            return

        self.curr_diff = self.left_area - self.right_area

        angle = int(self.curr_diff * PG + (self.curr_diff-self.last_diff) * PD)

        if self.turning:
            if self.track_dir == -1:
                if self.angleReached(90) or self.max_orange_area >= LINE_THRESH:
                    self.turning = False
                    angle = 0
                    self.turn_count += 1
                else:
                    angle = -MAX_TURN_DEGREE
            elif self.track_dir == 1:
                if self.angleReached(270) or self.max_blue_area >= LINE_THRESH:
                    self.turning = False
                    angle = 0
                    self.turn_count += 1
                else:
                    angle = MAX_TURN_DEGREE
                
        elif self.track_dir == 0:
            if self.left_area < WALL_THRESH:
                self.track_dir = -1
                self.turning = True
                self.turning_start_angle = self.current_angle
            elif self.right_area < WALL_THRESH:
                self.track_dir = 1
                self.turning = True
                self.turning_start_angle = self.current_angle
            elif self.max_orange_area >= LINE_THRESH:
                self.track_dir = 1
                self.turning = True
                self.turning_start_angle = self.current_angle
            elif self.max_blue_area >= LINE_THRESH:
                self.track_dir = -1
                self.turning = True
                self.turning_start_angle = self.current_angle


        elif self.track_dir == 1:
            if self.max_orange_area >= LINE_THRESH:
                self.turning = True
                self.turning_start_angle = self.current_angle
            if self.right_area < WALL_THRESH:
                self.turning = True
                self.turning_start_angle = self.current_angle

        elif self.track_dir == -1:
            if self.max_blue_area >= LINE_THRESH:
                self.turning = True
                self.turning_start_angle = self.current_angle
            if self.left_area < WALL_THRESH:
                self.turning = True
                self.turning_start_angle = self.current_angle

        self.servo = angle
        self.dc = self.speed

        self.last_diff = self.curr_diff

        

def main(args=None):
    rclpy.init(args=args)
    navigate_node = NavigateNode()
    rclpy.spin(navigate_node)
    navigate_node.destroy_node()
    rclpy.shutdown()
if __name__ == "__main__":
    main()
