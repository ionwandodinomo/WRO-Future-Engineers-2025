import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from led_controller import ros_robot_controller_sdk as rrc

class LEDController(Node):
    def __init__(self):
        super().__init__('led_controller_node')
        self.board = rrc.Board()
        self.get_logger().info('Turning off both RGB LEDs')
        self.board.set_rgb([[1, 0, 0, 0], [2, 0, 0, 0]])
        
        self.srv = self.create_service(Twist, 'LED_command', self.run_custom)
        self.get_logger().info('Service Server Ready: Waiting for requ ests...')

        
    def run_custom(self,msg):
        colour = [msg.r,msg.g,msg.b]
        self.get_logger().info(f"Setting LEDs to custom colour")
        self.board.set_rgb([[1, *colour], [2, *colour]])


def main(args=None):
    rclpy.init(args=args)
    led_node = LEDController()
    led_node.spin()
    rclpy.shutdown()

if __name__ == '__main__':
    main()