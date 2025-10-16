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
CONF_THRESHOLD = 80
TARGET_DIST = 500  # mm

ser = serial.Serial(PORT, BAUD, timeout=0.1)
buffer = bytearray()
lidar_lock = threading.Lock()

def circular_diff_deg(a, b):
    d = a - b
    d = (d + 180) % 360 - 180
    return d
"""
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
"""


latest_dist   = np.full(360, np.inf, dtype=np.float32)   # mm
latest_conf   = np.zeros(360, dtype=np.uint8)            # confidence
latest_ts     = np.zeros(360, dtype=np.float64)          # timestamp
lidar_lock    = threading.Lock()

CONF_THRESHOLD = 20
buffer = bytearray()

def interpolate_angles(start_angle, end_angle, n):
    """Fast linear interpolation of angles across a packet."""
    step = (end_angle - start_angle) / (n - 1)
    return [start_angle + i * step for i in range(n)]

def store_lidar_points(parsed):
    angles = interpolate_angles(parsed["start_angle"], parsed["end_angle"], len(parsed["points"]))
    ts = parsed["timestamp"]

    with lidar_lock:
        for (dist, conf), angle in zip(parsed["points"], angles):
            angle_key = int(round(angle)) % 360
            if conf >= CONF_THRESHOLD or latest_conf[angle_key] == 0:
                latest_dist[angle_key] = dist
                latest_conf[angle_key] = conf
                latest_ts[angle_key]   = ts
"""
def get_latest_distance(angle_query, tolerance=2):
    angle_key = int(round(angle_query)) % 360
    with lidar_lock:
        if latest_conf[angle_key] > 0:
            return {
                "distance": float(latest_dist[angle_key]),
                "confidence": int(latest_conf[angle_key]),
                "angle": angle_key,
                "timestamp": latest_ts[angle_key],
            }

        # Search nearby angles if no exact match
        candidates = [
            (abs((k - angle_key + 180) % 360 - 180), k)
            for k in range(360)
            if latest_conf[k] > 0
        ]
    if not candidates:
        return None
    diff, k = min(candidates, key=lambda x: x[0])
    if diff > tolerance:
        return None
    return {
        "distance": float(latest_dist[k]),
        "confidence": int(latest_conf[k]),
        "angle": k,
        "timestamp": latest_ts[k],
    }
"""

# usage
#right_filter = MedianFilter(5)
#smoothed = right_filter.update(right["distance"])
def get_latest_distance(angle_query, tolerance=2, window=2):
    """
    Query LiDAR distance at angle window degrees, return median of values.
    Falls back to nearest within tolerance if no exact match.
    """
    values = []
    for da in range(-window, window+1):
        angle_key = int(round(angle_query + da)) % 360
        with lidar_lock:
            if latest_conf[angle_key] > 0:
                values.append(latest_dist[angle_key])

    if values:
        return {"distance":float(np.median(values))}

    # fallback: search closest key within tolerance
    angle_key = int(round(angle_query)) % 360
    with lidar_lock:
        candidates = [
            (abs((k - angle_key + 180) % 360 - 180), latest_dist[k])
            for k in range(360) if latest_conf[k] > 0
        ]
    if not candidates:
        return None
    diff, dist = min(candidates, key=lambda x: x[0])
    return float(dist) if diff <= tolerance else None

def lidar_thread():
    global buffer
    while True:
        data = ser.read(256)   # larger read = fewer syscalls
        if not data:
            continue
        buffer.extend(data)

        while len(buffer) >= PACKET_LEN:
            idx = find_packet_start(buffer)
            if idx == -1:
                # keep only last PACKET_LEN-1 bytes to avoid junk buildup
                del buffer[:-PACKET_LEN+1]
                break

            if len(buffer) - idx < PACKET_LEN:
                break

            packet = buffer[idx:idx+PACKET_LEN]
            del buffer[:idx+PACKET_LEN]

            parsed = parse_packet(packet)
            if parsed:
                store_lidar_points(parsed)

# Start background LiDAR parser
threading.Thread(target=lidar_thread, daemon=True).start()
# PID variables
error_sum = 0
last_error = 0
Kp, Ki, Kd = 0.3, 0.0, 0.1
initial_yaw_deg = None
offset_deg = 0
passed = False
angle = MID_SERVO
wall = False
count = 0
flag = False

state = "follow_wall"
#right_filter = MedianFilter(size=5)
#front_filter = MedianFilter(size=5)
try:
    while True:
        
        yaw_deg = get_yaw_deg()
        if yaw_deg is not None:
            # Set initial yaw reference
            if initial_yaw_deg is None:
                initial_yaw_deg = yaw_deg
                
                # Compute offset from initial heading
            offset_deg = circular_diff_deg(yaw_deg, initial_yaw_deg)
            #print(offset_deg)
        else:
            print("Waiting for IMU data...")
            continue
        
#         angle_at = (0.0 + offset_deg + 360)%360
#         right = get_latest_distance(angle_at)
#         front = get_latest_distance(270.0)
        if state == "follow_wall":
            raw_right = get_latest_distance((0.0 + offset_deg + 360) % 360)
            raw_front = get_latest_distance(270.0)
        else:
            raw_right = get_latest_distance(0.0)
            raw_front = get_latest_distance(270.0)

#         right = right_filter.update(raw_right["distance"])
#         front = front_filter.update(raw_front["distance"])
        right = raw_right
        front = raw_front
        if right is None:
            continue  # skip if no valid reading
        #print(state, offset_deg, front["distance"])#,right["confidence"]) if front else None)
        print(front["distance"] if front else None)
        if state == "follow_wall":
            #print(angle)
        
            error = TARGET_DIST - right["distance"]
            error_sum += error  # Just sum per iteration (integral)
            d_error = error - last_error  # Difference per iteration (derivative)

            pid_output = int(Kp * error + Ki * error_sum + Kd * d_error)
            last_error = error

            # Clamp steering angle
            #front = get_latest_distance(270.0)
#             if right and 480 <= right["distance"] <= 520:
#                 Kp, Ki, Kd = 0.15, 0.0, 0.05
            if right and right["distance"] <= 380:
                flag = True
                angle = 0
    

            else:
                if flag:
                    state = "turn_90"
                    board.pwm_servo_set_position(0.1, [[1, pwm(MID_SERVO)]])
                    board.pwm_servo_set_position(0.1, [[2, 1500]])
                    time.sleep(1)
                    #print(front["distance"])
                    #break
                    continue
                
                angle = max(-MAX_TURN_DEGREE, min(MAX_TURN_DEGREE, pid_output))
            # Actuate motors
            board.pwm_servo_set_position(0.1, [[1, pwm(MID_SERVO + angle)]])
            board.pwm_servo_set_position(0.1, [[2, 1603]])  # DC motor
            #print()
        elif state == "adj":
            raw_front = get_latest_distance((270.0 + offset_deg + 360) % 360)

            #front = front_filter.update(raw_front["distance"])
            print(front["distance"])
            if front and front["distance"] <= 1470:
                state = "turn_90"
                board.pwm_servo_set_position(0.1, [[1, pwm(MID_SERVO)]])
                board.pwm_servo_set_position(0.1, [[2, 1500]])
                time.sleep(0.5)
            board.pwm_servo_set_position(0.1, [[2, 1590]])
        elif state == "turn_90":
            if abs(offset_deg) >= 78:
                state = "straight"
                board.pwm_servo_set_position(0.1, [[1, pwm(MID_SERVO)]])
                board.pwm_servo_set_position(0.1, [[2, 1500]])  # DC motor
                
            board.pwm_servo_set_position(0.1, [[1, pwm(MID_SERVO-MAX_TURN_DEGREE)]])
            board.pwm_servo_set_position(0.1, [[2, 1595]])  # DC motor
        elif state == "straight":
            if front and front["distance"] <= 150:
                state = "turn_in"
                board.pwm_servo_set_position(0.1, [[1, pwm(MID_SERVO)]])
                board.pwm_servo_set_position(0.1, [[2, 1500]])  # DC motor
            board.pwm_servo_set_position(0.1, [[1, pwm(MID_SERVO)]])
            board.pwm_servo_set_position(0.1, [[2, 1595]])  # DC motor
        elif state == "turn_in":
            front = get_latest_distance((270.0+offset_deg+360)%360)
            if abs(offset_deg) <= 1 or (front and front["distance"] <= 90):
                state = "back_fix"
                board.pwm_servo_set_position(0.1, [[1, pwm(MID_SERVO)]])
                board.pwm_servo_set_position(0.1, [[2, 1500]])  # DC motor
                
            board.pwm_servo_set_position(0.1, [[1, pwm(MID_SERVO+MAX_TURN_DEGREE+5)]])
            board.pwm_servo_set_position(0.1, [[2, 1595]])  # DC motor
        elif state == "back_fix":
            if abs(offset_deg) <= 1 or (front and front["distance"] >= 125):
                state = "back"
                board.pwm_servo_set_position(0.1, [[1, pwm(MID_SERVO)]])
                board.pwm_servo_set_position(0.1, [[2, 1500]])  # DC motor
                break
            board.pwm_servo_set_position(0.1, [[1, pwm(MID_SERVO-MAX_TURN_DEGREE)]])
            board.pwm_servo_set_position(0.1, [[2, 1390]])  # DC motor
        elif state == "back":
            if front and front["distance"] >= 120:
                board.pwm_servo_set_position(0.1, [[1, pwm(MID_SERVO)]])
                board.pwm_servo_set_position(0.1, [[2, 1500]])  # DC motor
                break
            board.pwm_servo_set_position(0.1, [[2, 1390]])

        time.sleep(0.01)
        
except Exception as e:
    board.pwm_servo_set_position(0.1, [[1, pwm(MID_SERVO)]])
    board.pwm_servo_set_position(0.1, [[2, 1500]])  # DC motor
    LED(board,(0,0,0))
    print(e)
    
board.pwm_servo_set_position(0.1, [[1, pwm(MID_SERVO)]])
board.pwm_servo_set_position(0.1, [[2, 1500]])
        



