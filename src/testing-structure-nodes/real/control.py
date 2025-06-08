import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from open_interfaces.srv import SendCommand
import ros_robot_controller_sdk as rcc
import time

board = rcc.Board()

def Const():
    MID_SERVO = 90

def pwm(degree):  # angle must be adjusted to pwm angle for servo
    return round(degree * 11.1 + 500)


class ControlNode(Node, Const):
    def __init__(self):
        super().__init__('control_node')
        self.started = False

        #self.subscription_running = self.create_subscription(Bool, '/states/running', self.shutdown, 10)

        self.srv = self.create_service(SendCommand, 'send_command', self.add_callback)
        self.get_logger().info('Service Server Ready: Waiting for requests...')

    def add_callback(self, request, response):
        response.sum = "changed"
 
        dcspeed = request.dc
        servo_angle = pwm(request.servo+self.MID_SERVO)

        self.get_logger().info(f'DC Motor: {dcspeed}')
        self.get_logger().info(f'Servo: {servo_angle}')

        board.pwm_servo_set_position(0.1, [[1, servo_angle]])
        time.sleep(0.1)
        
        board.pwm_servo_set_position(0.1, [[2, dcspeed]])
        time.sleep(0.1)

        return response



    def shutdown(self, msg):
        if msg.data == False:
            if self.started:
                #stop dc straight out motor
                super.destroy_node()

        else:
            self.started = True

    

def main():
    rclpy.init()
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()