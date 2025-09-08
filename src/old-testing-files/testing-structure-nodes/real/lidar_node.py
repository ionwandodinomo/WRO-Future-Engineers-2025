import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import struct
import time
from gpiozero import DigitalOutputDevice

PORT = "/dev/ttyAMA0"
BAUD = 230400
PACKET_HEADER = 0x54
PACKET_LEN = 47
LIDAR_POWER_PIN = 17

class LD19Node(Node):
    def __init__(self):
        super().__init__('ld19_node')

        # Power on LD19
        self.lidar_power = DigitalOutputDevice(LIDAR_POWER_PIN)
        self.lidar_power.on()
        time.sleep(1.5)
        self.get_logger().info("LD19 powered ON.")

        # Serial setup
        self.ser = serial.Serial(PORT, BAUD, timeout=0.1)
        self.buffer = bytearray()

        # ROS publisher
        self.publisher_ = self.create_publisher(Float32MultiArray, 'ld19/angle_distance', 10)
        self.timer = self.create_timer(0.01, self.read_lidar)

    def find_packet_start(self, buffer):
        for i in range(len(buffer) - 1):
            if buffer[i] == 0x54 and (buffer[i+1] & 0xFF) == 0x2C:
                return i
        return -1

    def parse_packet(self, packet):
        if len(packet) != PACKET_LEN:
            return None
        speed = struct.unpack_from('<H', packet, 2)[0] / 64.0
        start_angle = struct.unpack_from('<H', packet, 4)[0] / 100.0
        measurements = []
        for i in range(12):
            offset = 6 + i * 3
            dist = struct.unpack_from('<H', packet, offset)[0]
            confidence = packet[offset + 2]
            measurements.append((dist, confidence))
        end_angle = struct.unpack_from('<H', packet, 42)[0] / 100.0
        timestamp = struct.unpack_from('<H', packet, 44)[0]
        crc = packet[46]
        return {
            "speed": speed,
            "start_angle": start_angle,
            "end_angle": end_angle,
            "timestamp": timestamp,
            "crc": crc,
            "points": measurements
        }

    def interpolate_angles(self, start, end, count):
        angle_range = (end - start + 360) % 360
        step = angle_range / (count - 1)
        return [(start + i * step) % 360 for i in range(count)]

    def read_lidar(self):
        data = self.ser.read(256)
        if data:
            self.buffer += data
            while True:
                idx = self.find_packet_start(self.buffer)
                if idx == -1 or len(self.buffer) - idx < PACKET_LEN:
                    break
                packet = self.buffer[idx:idx+PACKET_LEN]
                self.buffer = self.buffer[idx+PACKET_LEN:]
                parsed = self.parse_packet(packet)
                if parsed:
                    angles = self.interpolate_angles(parsed["start_angle"], parsed["end_angle"], 12)
                    for (dist, conf), angle in zip(parsed["points"], angles):
                        if abs(angle - 0.0) < 0.5:  # publish around 0
                            msg = Float32MultiArray()
                            msg.data = [angle, float(dist)]
                            self.publisher_.publish(msg)
                            self.get_logger().info(f"Published: angle={angle:.2f}, dist={dist}")
                            break  # publish only the first near-0 point

def main(args=None):
    rclpy.init(args=args)
    node = LD19Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.lidar_power.off()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()