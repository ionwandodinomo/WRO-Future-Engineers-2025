import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from challenge import ros_robot_controller_sdk as rrc

class LEDController(Node):
    def __init__(self):
        super().__init__('led_controller_node')
        self.board = rrc.Board()
        self.get_logger().info('Turning off both RGB LEDs')
        self.board.set_rgb([[1, 0, 0, 0], [2, 0, 0, 0]])
        
        self.srv = self.create_subscription(Int32MultiArray, 'LED_command', self.run_custom,10)
        self.get_logger().info('Service Server Ready: Waiting for requ ests...')

        
    def run_custom(self,msg):
        colour = [msg[1],msg[2],msg[3]]
        if msg[0]==0:
            self.get_logger().info(f"Setting both LEDs to custom colour")
            self.board.set_rgb([[1, *colour], [2, *colour]])

        elif msg[0]==1:
            self.get_logger().info(f"Setting LED1 to custom colour")
            self.board.set_rgb([[1, *colour], [2, 0,0,0]])
        elif msg[0]==2:
            self.get_logger().info(f"Setting LED2 to custom colour")
            self.board.set_rgb([[1, 0,0,0], [2, *colour]])


def main(args=None):
    rclpy.init(args=args)
    led_node = LEDController()
    rclpy.spin(led_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()