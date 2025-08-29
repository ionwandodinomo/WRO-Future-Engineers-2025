# class 12
import serial
import struct
import time

PORT = "/dev/ttyAMA0"
BAUD = 230400

PACKET_HEADER = 0x54
PACKET_LEN = 47

def find_packet_start(buffer):
    """Find the start index of a valid packet in buffer."""
    for i in range(len(buffer) - 1):
        if buffer[i] == 0x54 and (buffer[i+1] & 0xFF) == 0x2C:
            return i
    return -1

def parse_packet(packet):
    if len(packet) != PACKET_LEN:
        return None

    speed = struct.unpack_from('<H', packet, 2)[0] / 64.0  # RPM
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

def interpolate_angles(start, end, count):
    """Return a list of interpolated angles from start to end over count points."""
    angle_range = (end - start + 360) % 360
    step = angle_range / (count - 1)
    return [(start + i * step) % 360 for i in range(count)]
