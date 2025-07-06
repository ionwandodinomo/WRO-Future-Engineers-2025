import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Bool
from challenge import ros_robot_controller_sdk as rcc
import time

board = rcc.Board()

MID_SERVO = 128

def pwm(degree):  # angle must be adjusted to pwm angle for servo
    return round(degree * 11.1 + 500)


class ControlNode(Node):
    def __init__(self):
        super().__init__('control')
        self.started = False

        #self.subscription_running = self.create_subscription(Bool, '/states/running', self.shutdown, 10)
        board.pwm_servo_set_position(0.1, [[1,  pwm(MID_SERVO)]])
        time.sleep(0.1)

        self.srv = self.create_subscription(Int32MultiArray, 'send_command', self.add_callback,10)
        self.get_logger().info('Service Server Ready: Waiting for requests...')

    def add_callback(self, request, response):
        
        dcspeed = request.data[1]
        servo_angle = pwm(request.data[0]+MID_SERVO)

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
    board.pwm_servo_set_position(0.1, [[1,  pwm(MID_SERVO)]])
    time.sleep(0.1)