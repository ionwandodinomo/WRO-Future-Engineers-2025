import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import RPi.GPIO as GPIO
from ros_robot_controller_msgs.msg import ButtonState
import time

BUTTON_PIN = 17  # Change to your actual GPIO pin

class ButtonStartNode(Node):
    def __init__(self):
        super().__init__('button_start_node')
        self.publisher_ = self.create_publisher(Bool, '/states/running', 10)

        self.subscription_button = self.create_subscription(
            ButtonState,
            "/ros_robot_controller/button",
            self.button_callback,
            10)

        self.started = False

        msg = Bool()
        msg.data = True
        self.publisher_.publish(self.started)

    def button_callback(self, msg:ButtonState):
        if msg.id == 1 and msg.state == 1 and not self.started:
            self.started = not self.started
            self.get_logger().info("Button 1 pressed.")

            msg = Bool()
            msg.data = self.started
            self.publisher_.publish(self.started)

        elif msg.id == 1 and msg.state == 1 and self.started:
            self.destroy_node()
            rclpy.shutdown()

    def destroy_node():
        super.destroy_node()



def main():
    rclpy.init()
    node = ButtonStartNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
