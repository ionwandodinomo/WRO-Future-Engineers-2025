import time
import rclpy
from rclpy.node import Node
from led_controller import ros_robot_controller_sdk as rrc

class LEDController(Node):
    def __init__(self):
        super().__init__('led_controller_node')
        self.board = rrc.Board()
        self.get_logger().info('Turning off both RGB LEDs')
        self.board.set_rgb([[1, 0, 0, 0], [2, 0, 0, 0]])
        self.run_sequence()

    def run_sequence(self):
        colors = [
            ("red", [255, 0, 0]),
            ("green", [0, 255, 0]),
            ("blue", [0, 0, 255]),
            ("yellow", [255, 255, 0]),
        ]

        for name, rgb in colors:
            self.get_logger().info(f"Setting LEDs to {name}")
            self.board.set_rgb([[1, *rgb], [2, *rgb]])
            time.sleep(1)

        self.get_logger().info("Turning off both RGB LEDs")
        self.board.set_rgb([[1, 0, 0, 0], [2, 0, 0, 0]])

def main(args=None):
    rclpy.init(args=args)
    led_node = LEDController()
    rclpy.shutdown()

if __name__ == '__main__':
    main()