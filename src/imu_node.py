# imu ros2 node + threading for real time data
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

# Threaded initialization
imu_node = None
rclpy_thread = None
def start_imu_listener():
    """start imu listener to stream data"""
    global imu_node, rclpy_thread
    rclpy.init()
    imu_node = IMUSubscriber()
    rclpy_thread = threading.Thread(target=lambda: rclpy.spin(imu_node), daemon=True)
    rclpy_thread.start()

def get_yaw_deg():
    """Get the latest yaw angle in degrees."""
    if imu_node:
        yaw_rad = imu_node.get_latest_yaw()
        return math.degrees(yaw_rad) if yaw_rad is not None else None

def stop_imu_listener():
    """stop imu listener and cleanup"""
    global imu_node
    if imu_node:
        imu_node.destroy_node()
    rclpy.shutdown()
