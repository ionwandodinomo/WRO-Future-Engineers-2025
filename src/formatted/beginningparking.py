import rclpy
import cv2
import numpy as np
from picamera2 import Picamera2
from gpiozero import DigitalOutputDevice
import ros_robot_controller_sdk as rcc
from helper import *
from CONSTS import *
from lidar_parser import *
from imu import *
import time
board = rcc.Board()
lidar_power = DigitalOutputDevice(LIDAR_POWER_PIN)
lidar_power.on()
time.sleep(1)
print("LD19 powered ON via GPIO 17.")
latest_points = {}  # angle (int) → (distance, confidence, timestamp)


debug = True

speed = 1605
last_pil_diff = 0
last_diff = 0
turning = False
turn_dir = 0
track_dir = 0
curr_diff = 0
error = 0
angle = 0
turn_count = 0
time_after_turn = 0
target = 0

CONF_THRESHOLD = 50

def store_latest_points(parsed):
    """Update the latest points dictionary with the newest packet if confidence is good."""
    angles = interpolate_angles(parsed["start_angle"], parsed["end_angle"], len(parsed["points"]))
    ts = parsed["timestamp"]

    for (dist, conf), angle in zip(parsed["points"], angles):
        angle_key = int(round(angle)) % 360

        # Only update if confidence is above threshold
        if conf >= CONF_THRESHOLD:
            latest_points[angle_key] = (dist, conf, ts)
        else:
            # keep old value if it exists
            if angle_key not in latest_points:
                latest_points[angle_key] = (dist, conf, ts)  # accept if nothing stored yet


def get_latest_distance(angle_query, tolerance=2):
    """
    Get the most recent distance for the given angle.
    `tolerance` = how many degrees off we can match.
    """
    angle_key = int(round(angle_query)) % 360

    # Try exact match
    if angle_key in latest_points:
        dist, conf, ts = latest_points[angle_key]
        return {"distance": dist, "confidence": conf, "angle": angle_key, "timestamp": ts}

    # Try within ±tolerance degrees
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
        board.pwm_servo_set_position(0.1, [[2, 1605]])
    elif direction == "reverse":
        board.pwm_servo_set_position(0.1, [[2, 1350]])

def stop_motion():
    """Stop all motion"""
    board.pwm_servo_set_position(0.1, [[1, pwm(MID_SERVO)]])
    board.pwm_servo_set_position(0.1, [[2, 1500]])

def set_steering(position):
    if position == "left":
        board.pwm_servo_set_position(0.1, [[1, pwm(MID_SERVO+MAX_TURN_DEGREE)]])
    elif position == "right":
        board.pwm_servo_set_position(0.1, [[1, pwm(MID_SERVO-MAX_TURN_DEGREE)]])
    elif position == "straight":
        board.pwm_servo_set_position(0.1, [[1, pwm(MID_SERVO)]])




"""if right_area > left_area and right_area > 2000:
    track_dir = -1
elif left_area > right_area and left_area > 2000:
    track_dir = 1"""
     
if track_dir != 0:
    # unparking code here
    #unpark(board,track_dir)
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    buffer = bytearray()
    imu_proc = start_imu_stream()
    in_vector_block = False
    last_yaw = None

    # Movement states
    state = "detect_wall"  # Other states: reverse_steer, reverse_straight, forward_out
    wall_side = None  # "left" or "right"


    try:
        while True:
            data = ser.read(256)
            if data:
                buffer += data
                while True:
                    idx = find_packet_start(buffer)
                    if idx == -1 or len(buffer) - idx < PACKET_LEN:
                        break
                    packet = buffer[idx:idx+PACKET_LEN]
                    buffer = buffer[idx+PACKET_LEN:]

                    parsed = parse_packet(packet)
                    if parsed:
                        store_latest_points(parsed)
            front = get_latest_distance(90.0)  # front center
            left = get_latest_distance(60.0)   # left-ish
            right = get_latest_distance(120.0) # right-ish
            line = read_imu_line(imu_proc)
            if line:
                if line.startswith("vector:"):
                    in_vector_block = True
                elif in_vector_block and line.startswith("z:"):
                    try:
                        z_rad = float(line.split(":", 1)[1].strip())
                        z_deg = math.degrees(z_rad)
                        last_yaw = z_deg
                        print(f"IMU yaw = {z_deg:.1f}")
                    except ValueError:
                        pass
                    in_vector_block = False

            # ---- State Machine ----
            if state == "detect_wall":
                if left and right:
                    if left["distance"] < 300:  # <30 cm → wall left
                        wall_side = "left"
                        state = "reverse_steer"
                        print("Wall detected on LEFT")
                    elif right["distance"] < 300:
                        wall_side = "right"
                        state = "reverse_steer"
                        print("Wall detected on RIGHT")

            elif state == "reverse_steer":
                if wall_side == "left":
                    set_steering("right")
                else:
                    set_steering("left")
                set_motion("reverse", speed=0.3)

                if front and front["distance"] > 500:  # >50cm front clearance
                    state = "reverse_straight"
                    print("Front clearance gained")

            elif state == "reverse_straight":
                set_steering("straight")
                set_motion("reverse", speed=0.3)

                if front and front["distance"] > 800:  # Enough front clearance
                    state = "forward_out"
                    print("Switch to forward")

            elif state == "forward_out":
                set_steering("straight")
                set_motion("forward", speed=0.4)

                # Use IMU to correct orientation if needed
                if abs(last_yaw - initial_yaw) < 5:
                    stop_motion()
                    print("Unparking complete")
                    break

            time.sleep(0.1)
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Stopping lidar + IMU loop…")
    finally:
        stop_imu_stream(imu_proc)
        ser.close()
        lidar_power.off()
        time.sleep(1)   