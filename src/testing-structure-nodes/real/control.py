import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import ros_robot_controller_sdk as rcc
import time

board = rcc.Board()

MID_SERVO = 128

def pwm(degree):  # angle must be adjusted to pwm angle for servo
    return round(degree * 11.1 + 500)


class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.started = False

        #self.subscription_running = self.create_subscription(Bool, '/states/running', self.shutdown, 10)

        self.srv = self.create_service(Twist, 'send_command', self.add_callback)
        self.get_logger().info('Service Server Ready: Waiting for requests...')

    def add_callback(self, request, response):
 
        dcspeed = request.dc
        servo_angle = pwm(request.servo+MID_SERVO)

        self.get_logger().info(f'DC Motor: {dcspeed}')
        self.get_logger().info(f'Servo: {servo_angle}')

        board.pwm_servo_set_position(0.1, [[1, servo_angle]])
        time.sleep(0.1)
        
        board.pwm_servo_set_position(0.1, [[2, dcspeed]])
        time.sleep(0.1)

        return response




    

def main():
    rclpy.init()
    node = ControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()