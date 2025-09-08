import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import serial
import struct
import time
from collections import deque
import threading
import math

PORT = "/dev/ttyAMA0"
BAUD = 230400
PACKET_HEADER = 0x54
PACKET_LEN = 47

angles = deque(maxlen=360)
distances = deque(maxlen=360)
lock = threading.Lock()

def find_packet_start(buffer):
    for i in range(len(buffer) - 1):
        if buffer[i] == 0x54 and (buffer[i+1] & 0xFF) == 0x2C:
            return i
    return -1

def parse_packet(packet):
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

def interpolate_angles(start, end, count):
    angle_range = (end - start + 360) % 360
    step = angle_range / (count - 1)
    return [(start + i * step) % 360 for i in range(count)]

fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='polar')
ax.set_theta_offset(np.pi/2)
ax.set_theta_direction(-1)
scatter = ax.scatter([], [], s=5, c=[], cmap='viridis', alpha=0.75)
ax.set_rmax(8000)
ax.grid(True)
ax.set_title("LiDAR Scanner - 5° Resolution", va='bottom')

try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
    buffer = bytearray()
except Exception as e:
    print(f"Error opening serial port: {e}")
    ser = None

def read_serial_data():
    global buffer, angles, distances
    
    if ser is None:
        while True:
            time.sleep(0.1)
            with lock:
                for angle in range(0, 360, 5):
                    angle_rad = np.radians(angle)
                    distance = 5000 + 3000 * np.sin(np.radians(angle * 4))
                    angles.append(angle_rad)
                    distances.append(distance)
        return
    
    while True:
        data = ser.read(ser.in_waiting or 1)
        if data:
            buffer.extend(data)
            
            while True:
                start_idx = find_packet_start(buffer)
                if start_idx == -1:
                    if len(buffer) > PACKET_LEN * 2:
                        buffer = bytearray()
                    break
                    
                if start_idx > 0:
                    buffer = buffer[start_idx:]
                    
                if len(buffer) < PACKET_LEN:
                    break
                    
                packet = buffer[:PACKET_LEN]
                buffer = buffer[PACKET_LEN:]
                
                result = parse_packet(packet)
                if result:
                    interpolated_angles = interpolate_angles(
                        result["start_angle"], 
                        result["end_angle"], 
                        12
                    )
                    
                    with lock:
                        for i, (dist, confidence) in enumerate(result["points"]):
                            if dist > 0:
                                angle_deg = interpolated_angles[i]
                                if angle_deg % 5 < 0.5 or angle_deg % 5 > 4.5:
                                    angle_rad = np.radians(angle_deg)
                                    angles.append(angle_rad)
                                    distances.append(dist)

def update_plot(frame):
    with lock:
        if angles and distances:
            angles_array = np.array(angles)
            distances_array = np.array(distances)
            
            scatter.set_offsets(np.c_[angles_array, distances_array])
            scatter.set_array(distances_array)
            
    return scatter,

thread = threading.Thread(target=read_serial_data, daemon=True)
thread.start()

ani = animation.FuncAnimation(fig, update_plot, interval=50, blit=True)

plt.tight_layout()
plt.show()