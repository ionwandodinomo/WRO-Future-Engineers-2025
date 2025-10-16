# imu_node.py
from geometry_msgs.msg import Vector3Stamped
import rclpy
from rclpy.node import Node
import threading
import math

class IMUSubscriber(Node):
    def __init__(self):
        super().__init__('imu_subscriber')
        self.latest_yaw_rad = None
        self.lock = threading.Lock()
        self.create_subscription(Vector3Stamped, '/imu/rpy/filtered', self.callback, 10)

    def callback(self, msg):
        z = msg.vector.z
        if z < 0:
            z += 2 * math.pi
        with self.lock:
            self.latest_yaw_rad = z

    def get_latest_yaw(self):
        with self.lock:
            return self.latest_yaw_rad


# Global holder
imu_node: IMUSubscriber = None

def create_imu_node():
    """Create the IMU node but don't spin it here."""
    global imu_node
    imu_node = IMUSubscriber()
    return imu_node

def get_yaw_deg():
    if imu_node:
        yaw_rad = imu_node.get_latest_yaw()
        return math.degrees(yaw_rad) if yaw_rad is not None else None
    return None

