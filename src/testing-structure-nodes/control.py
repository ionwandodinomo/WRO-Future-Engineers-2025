import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

def Const():
    MID_SERVO = 90

def pwm(degree):  # angle must be adjusted to pwm angle for servo
    return round(degree * 11.1 + 500)


class ControlNode(Node, Const):
    def __init__(self):
        super().__init__('control_node')
        self.started = False


        self.subscription_cmd = self.create_subscription(Twist, 'cmd', self.control_callback, 10)

        self.subscription_running = self.create_subscription(Bool, '/states/running', self.shutdown, 10)


    def control_callback(self, msg):
        dcspeed = msg.dc
        servo_angle = msg.servo
        self.get_logger().info(f'DC Motor: {dcspeed}')
        self.get_logger().info(f'Servo: {servo_angle}')

        board = pwm(servo_angle+self.MID_SERVO)
        #i forgots oppysies popositssx

    def shutdown(self, msg):
        if msg.data == False:
            if self.started:
                #stop dc straight out motor
                super.destroy_node()

        else:
            self.started = True