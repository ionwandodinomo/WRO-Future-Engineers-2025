import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float32
import math

def radiansToDegrees(rad):
    degree = math.degrees(rad) % 360
    return degree

class IMUNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        self.subscription = self.create_subscription(
            Vector3Stamped,
            "/imu/rpy/filtered",
            self.imu_callback,
            10)
        
        self.publisher_ = self.create_publisher(Float32, 'imu', 10)
        self.timer = self.create_timer(0.1, self.publish_imu)
        
        self.currentAngle = 0.0

    def imu_callback(self, msg):
        self.currentAngle = radiansToDegrees(msg.vector.z)

    def publish_imu(self):
        self.publisher_.publish(self.currentAngle)
