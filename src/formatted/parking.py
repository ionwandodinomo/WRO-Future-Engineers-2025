import cv2
import numpy as np
from picamera2 import Picamera2
from gpiozero import DigitalOutputDevice
import ros_robot_controller_sdk as rcc
from helper import *
from CONSTS import *
from lidar_parser import *
import math
import time
import serial
from imu_node import start_imu_listener, get_yaw_deg, stop_imu_listener

board = rcc.Board()
lidar_power = DigitalOutputDevice(LIDAR_POWER_PIN)
lidar_power.on()
time.sleep(1)
print("LD19 powered ON via GPIO 17.")

latest_points = {}
CONF_THRESHOLD = 0
speed = 1605
track_dir = 1

LED(board, (0, 255, 0))

def unsigned_circular_diff_deg(a, b):
    d = a - b
    d = (d + 180) % 360 - 180
    return abs(d)

def store_lidar_points(parsed):
    angles = interpolate_angles(parsed["start_angle"], parsed["end_angle"], len(parsed["points"]))
    ts = parsed["timestamp"]
    for (dist, conf), angle in zip(parsed["points"], angles):
        angle_key = int(round(angle)) % 360
        if conf >= CONF_THRESHOLD or angle_key not in latest_points:
            latest_points[angle_key] = (dist, conf, ts)

def get_wall_side(left, right):
    if left and left["distance"] < 500:
        return "left"
    elif right and right["distance"] < 500:
        return "right"
    return None
def get_latest_distance(angle_query, tolerance=2):
    angle_key = int(round(angle_query)) % 360
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


def set_motion(direction):
    if direction == "forward":
        board.pwm_servo_set_position(0.1, [[2, 1500]])  # 1590
    elif direction == "reverse":
        board.pwm_servo_set_position(0.1, [[2, 1500]])  # 1390

def stop_motion():
    board.pwm_servo_set_position(0.1, [[1, pwm(MID_SERVO)]])
    board.pwm_servo_set_position(0.1, [[2, 1500]])

def set_steering(position):
    if position == "left":
        board.pwm_servo_set_position(0.1, [[1, pwm(MID_SERVO - MAX_TURN_DEGREE)]])
    elif position == "right":
        board.pwm_servo_set_position(0.1, [[1, pwm(MID_SERVO)]])
    elif position == "straight":
        board.pwm_servo_set_position(0.1, [[1, pwm(MID_SERVO)]])
        
if track_dir != 0:
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    buffer = bytearray()

    start_imu_listener()
    initial_yaw_deg = None
    offset_deg = 0
    state = "detect_wall"
    wall_side = None
    dist_from_front = 0

    print("starting reading")
    time.sleep(1)

    try:
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

            front = get_latest_distance(270.0)
            left = get_latest_distance(180.0)
            right = get_latest_distance(0.0)

            yaw_deg = get_yaw_deg()
            if yaw_deg is not None:
                if initial_yaw_deg is None:
                    initial_yaw_deg = yaw_deg
                offset_deg = unsigned_circular_diff_deg(yaw_deg, initial_yaw_deg)
                print(f"Offset: {offset_deg:.2f} | Yaw: {yaw_deg:.2f}")
            else:
                print("Waiting for IMU data...")

            if front and left and right:
                if state == "detect_wall":
                    wall_side = get_wall_side(left, right)
                    if wall_side:
                        state = "front_turn"
                    dist_from_front = front["distance"]

                elif state == "front_turn":
                    set_steering(wall_side)
                    set_motion("forward")
                    if any([front["distance"] < 50, left["distance"] < 50, right["distance"] < 50]):
                        stop_motion()
                        time.sleep(2)
                        state = "reverse_turn"

                elif state == "reverse_turn":
                    set_steering("right" if wall_side == "left" else "left")
                    set_motion("reverse")
                    if offset_deg > 40:
                        stop_motion()
                        state = "turn_til_90"

                elif state == "turn_til_90":
                    set_steering(wall_side)
                    set_motion("forward")
                    if offset_deg > 0:
                        stop_motion()
                        break

            time.sleep(0.02)

    except KeyboardInterrupt:
        pass
    finally:
        stop_imu_listener()
        ser.close()
        lidar_power.off()
        time.sleep(1)

LED(board, (0, 0, 0))
