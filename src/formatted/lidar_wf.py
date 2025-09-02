import cv2
import numpy as np
from picamera2 import Picamera2
from gpiozero import DigitalOutputDevice
from helper import *
from CONSTS import *
from lidar_parser import *
import math
import threading
import ros_robot_controller_sdk as rcc
import time
import serial
from imu_node import start_imu_listener, get_yaw_deg, stop_imu_listener

board = rcc.Board()
start_imu_listener()

lidar_power = DigitalOutputDevice(LIDAR_POWER_PIN)
lidar_power.on()
time.sleep(1)
print("LD19 powered ON via GPIO 17.")

latest_points = {}
CONF_THRESHOLD = 0
TARGET_DIST = 300  # 30cm

ser = serial.Serial(PORT, BAUD, timeout=0.1)
buffer = bytearray()
lidar_lock = threading.Lock()

def circular_diff_deg(a, b):
    d = a - b
    d = (d + 180) % 360 - 180
    return d

def store_lidar_points(parsed):
    angles = interpolate_angles(parsed["start_angle"], parsed["end_angle"], len(parsed["points"]))
    ts = parsed["timestamp"]
    with lidar_lock:
        for (dist, conf), angle in zip(parsed["points"], angles):
            angle_key = int(round(angle)) % 360
            if conf >= CONF_THRESHOLD or angle_key not in latest_points:
                latest_points[angle_key] = (dist, conf, ts)

def get_latest_distance(angle_query, tolerance=2):
    angle_key = int(round(angle_query)) % 360
    with lidar_lock:
        if angle_key in latest_points:
            dist, conf, ts = latest_points[angle_key]
            return {"distance": dist, "confidence": conf, "angle": angle_key, "timestamp": ts}
        candidates = [
            (abs(k - angle_key), k, latest_points[k])
            for k in latest_points
            if abs((k - angle_key + 180) % 360 - 180) <= tolerance
        ]
    if not candidates:
        return None
    _, k, (dist, conf, ts) = min(candidates, key=lambda x: x[0])
    return {"distance": dist, "confidence": conf, "angle": k, "timestamp": ts}

def lidar_thread():
    global buffer
    while True:
        data = ser.read(256)
        if data:
            buffer += data
            while True:
                idx = find_packet_start(buffer)
                if idx == -1 or len(buffer) - idx < PACKET_LEN:
                    break
                packet = buffer[idx:idx + PACKET_LEN]
                buffer = buffer[idx + PACKET_LEN:]
                parsed = parse_packet(packet)
                if parsed:
                    store_lidar_points(parsed)

# Start LiDAR parsing in a background thread
threading.Thread(target=lidar_thread, daemon=True).start()
# PID variables
error_sum = 0
last_error = 0
Kp, Ki, Kd = 0.5, 0.0, 0.1
initial_yaw_deg = None
offset_deg = 0

while True:
    
    yaw_deg = get_yaw_deg()
    if yaw_deg is not None:
        # Set initial yaw reference
        if initial_yaw_deg is None:
            initial_yaw_deg = yaw_deg
            
            # Compute offset from initial heading
        offset_deg = circular_diff_deg(yaw_deg, initial_yaw_deg)
        print(offset_deg)
    else:
        print("Waiting for IMU data...")
        continue
    
    angle_at = (0.0 + offset_deg + 360)%360
    right = get_latest_distance(angle_at)
    if right is None:
        continue  # skip if no valid reading

    
    error = TARGET_DIST - right["distance"]
    error_sum += error  # Just sum per iteration (integral)
    d_error = error - last_error  # Difference per iteration (derivative)

    pid_output = int(Kp * error + Ki * error_sum + Kd * d_error)
    last_error = error

    # Clamp steering angle
    angle = max(-MAX_TURN_DEGREE, min(MAX_TURN_DEGREE, pid_output))
    print(f"Right Distance: {right['distance']}", angle_at, angle)

    # Actuate motors
    board.pwm_servo_set_position(0.1, [[1, pwm(MID_SERVO + angle)]])
    board.pwm_servo_set_position(0.1, [[2, 1603]])  # DC motor
