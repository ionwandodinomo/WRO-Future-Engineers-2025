
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
import sys
e = threading.Thread(target=start_nodes, kwargs={"IMU": True}, daemon=True).start()
board = rcc.Board()
lidar_power = DigitalOutputDevice(LIDAR_POWER_PIN)
lidar_power.on()
time.sleep(3)
#red is right
#green is left
PARKING = False
debug = False
if len(sys.argv) > 1 and sys.argv[1] == "debug":
    debug = True
    print("debuging")
    
MAX_TURN_COUNT = 1
NEXT_TURN = False
    

ATC = 45
latest_points = {}
CONF_THRESHOLD = 80
TARGET_DIST = 450  # mm

ser = serial.Serial(PORT, BAUD, timeout=0.1)
buffer = bytearray()
lidar_lock = threading.Lock()

def circular_diff_deg(a, b):
    d = a - b
    d = (d + 180) % 360 - 180
    return d

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
    return {"distance":float(dist)} if diff <= tolerance else None

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

initial_yaw_deg = None
state = "detect_wall"
direction = None
front = None
left = None
right = None
side = None
sign = 0
base_side_angle = None
#right_filter = MedianFilter(size=5)
#front_filter = MedianFilter(size=5)


while True:
    LED(board,(255,0,0))
    if check_node_status():
        print("READy")
        LED(board,(255,255,0))
        listen_to_button_events(board)
        print("STARTING OBSTACLE CHALLENGE CODE")
        LED(board,(255,255,255))
        time.sleep(0.5)
        LED(board,(0,0,0))
        time.sleep(0.2)
        LED(board,(255,255,255))
        time.sleep(0.5)
        LED(board,(0,0,0))
        time.sleep(0.2)
        LED(board,(0,255,0))
        time.sleep(0.5)
        LED(board,(0,0,0))
        break

if not PARKING:
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
        
        if state == "detect_wall":
            front = get_latest_distance((270.0 + offset_deg + 360) % 360)
            left = get_latest_distance(180.0)
            right = get_latest_distance(0.0)
        elif state == "turn_out":
            front = get_latest_distance((270.0 + offset_deg + 360) % 360)
            side = get_latest_distance(base_side_angle)
            print(state,front["distance"], side["distance"], offset_deg)
        else:
            front = get_latest_distance(270.0)
            side = get_latest_distance(base_side_angle)
            print(state,front["distance"], side["distance"], offset_deg)
            
            
        if not front:
            continue
        
        if state == "detect_wall":
            if left and left["distance"] < 300:
                direction = "left"
                base_side_angle = 180.0
                sign = -1
            elif right and right["distance"] < 300:
                direction = "right"
                base_side_angle = 280.0
                sign = 1
            else:
                print("NOT IN PARKING")
                break
            
            state = "turn_out"
            print(direction)
        
        elif state == "turn_out":
            if abs(offset_deg) >= 80:
                state = "front"
                board.pwm_servo_set_position(0.1, [[2, 1500]])
                board.pwm_servo_set_position(0.1, [[1, pwm(MID_SERVO)]])
            elif front["distance"] <= 80:
                board.pwm_servo_set_position(0.1, [[2, 1390]])
                board.pwm_servo_set_position(0.1, [[1, pwm(MID_SERVO)]])
                time.sleep(0.7)
                board.pwm_servo_set_position(0.1, [[2, 1500]])
            else:
                board.pwm_servo_set_position(0.1, [[2, 1590]])
                angle = MID_SERVO+(MAX_TURN_DEGREE*sign)
                board.pwm_servo_set_position(0.1, [[1, pwm(angle)]])
        elif state == "front":
            if front["distance"] <= 600:
                state = "straighten"
                board.pwm_servo_set_position(0.1, [[2, 1500]])
                board.pwm_servo_set_position(0.1, [[1, pwm(MID_SERVO)]])
            else:
                board.pwm_servo_set_position(0.1, [[2, 1590]])
                board.pwm_servo_set_position(0.1, [[1, pwm(MID_SERVO)]])
                
        elif state == "straighten":
            if abs(offset_deg) <= 1:
                state = "back"
                board.pwm_servo_set_position(0.1, [[2, 1500]])
                board.pwm_servo_set_position(0.1, [[1, pwm(MID_SERVO)]])
            elif front["distance"] <= 930 and abs(offset_deg)<= 25 and direction == "right":
                if abs(offset_deg) <= 1:
                    state = "back"
                    board.pwm_servo_set_position(0.1, [[2, 1500]])
                    board.pwm_servo_set_position(0.1, [[1, pwm(MID_SERVO)]])
                else:
                    board.pwm_servo_set_position(0.1, [[2, 1380]])
                    board.pwm_servo_set_position(0.1, [[1, pwm(MID_SERVO)]])
                    time.sleep(1)
            else:
                board.pwm_servo_set_position(0.1, [[2, 1590]])
                angle = MID_SERVO+(MAX_TURN_DEGREE*-sign)
                board.pwm_servo_set_position(0.1, [[1, pwm(angle)]])
        elif state == "back":
            if direction == "right":
                board.pwm_servo_set_position(0.1, [[2, 1380]])
                board.pwm_servo_set_position(0.1, [[1, pwm(MID_SERVO-5)]])
                time.sleep(1.8)
            else:
                board.pwm_servo_set_position(0.1, [[2, 1380]])
                board.pwm_servo_set_position(0.1, [[1, pwm(MID_SERVO-5)]])
                time.sleep(1.5)
            break
        
        time.sleep(0.05)

lidar_power.off()

board.pwm_servo_set_position(0.1, [[2, 1500]])
board.pwm_servo_set_position(0.1, [[1, pwm(MID_SERVO)]])
speed = 1605
button_id = 0
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
actions_to_straight = 0
button_pressed = False

def monitor_button():
    global button_pressed
    while True:
        output = process.stdout.readline()
        if output:
            line = output.strip()
            if line.startswith("id:"):
                button_id = int(line.split(":")[1].strip())
            elif line.startswith("state:"):
                state = int(line.split(":")[1].strip())
                if button_id in (1,2) :
                    if state == 1:
                        button_pressed = True
                        print("Button pressed")
                        
command = 'source /home/ubuntu/.zshrc && ros2 topic echo /ros_robot_controller/button'
process = subprocess.Popen(
        ['docker', 'exec', '-u', 'ubuntu', '-w', '/home/ubuntu', 'MentorPi', '/bin/zsh', '-c', command],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True
)
button_id = 0
button_pressed = False
turned_on = False
threading.Thread(target=monitor_button, daemon=True).start()

picam2 = Picamera2()
picam2.preview_configuration.main.size = (640,480)
picam2.preview_configuration.main.format = "RGB888"
print(picam2.preview_configuration.controls.FrameRate)
picam2.preview_configuration.controls.FrameRate = 30
picam2.set_controls({"Brightness": 0.05})
print(picam2.preview_configuration.controls.FrameRate)
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

p = False

while not button_pressed and not PARKING:
    s = time.time()
    frame = picam2.capture_array()
    frame = cv2.resize(frame, (640, 480))
    """hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Split channels
    h, s, v = cv2.split(hsv)

    # Increase saturation (clip values to 255)
    s = cv2.add(s, 50)  # You can adjust this value (e.g., 50-150) for intensity

    # Merge back
    hsv = cv2.merge([h, s, v])

    # Convert back to BGR
    frame = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    img_lab = cv2.cvtColor(frame, cv2.COLOR_BGR2Lab)
    img_lab = cv2.GaussianBlur(img_lab, (7, 7), 0)"""
    img_lab = preprocess_frame(frame)
    
    if turn_count >= MAX_TURN_COUNT:
        if direction == "left":
            NEXT_TURN = True
        print("READY TO END")
        actions_to_straight += 1
        lidar_power.on()
        if actions_to_straight >= ATC:
            board.pwm_servo_set_position(0.1, [[1, 1500]])
            board.pwm_servo_set_position(0.1, [[2, 1500]])
            time.sleep(1)
            actions_to_straight = -100
            p = True
            if not NEXT_TURN:
                break
        
    

    left_contours_top = findContours(img_lab, rBlack, ROI_LEFT_TOP)
    right_contours_top = findContours(img_lab, rBlack, ROI_RIGHT_TOP)
    left_contours_bot = findContours(img_lab, rBlack, ROI_LEFT_BOT)
    right_contours_bot = findContours(img_lab, rBlack, ROI_RIGHT_BOT)

    contours_blue1 = findContours(img_lab, rBlue, ROI_LINE1)

    contours_orange1 = findContours(img_lab, rOrange, ROI_LINE1)
    contours_blue2 = findContours(img_lab, rBlue, ROI_LINE2)

    contours_orange2 = findContours(img_lab, rOrange, ROI_LINE2)

    
    contours_red = findContours(img_lab, rRed, ROI_PILLAR)

    contours_green = findContours(img_lab, rGreen, ROI_PILLAR)
    max_left_top_contour = sortContourShapes(left_contours_top)[0]
    left_area_top = cv2.contourArea(max_left_top_contour) if is_valid_contour(max_left_top_contour) else 0

    max_right_top_contour = sortContourShapes(right_contours_top)[0]
    right_area_top = cv2.contourArea(max_right_top_contour) if is_valid_contour(max_right_top_contour) else 0

    max_left_bot_contour = sortContourShapes(left_contours_bot)[0]
    left_area_bot = cv2.contourArea(max_left_bot_contour) if is_valid_contour(max_left_bot_contour) else 0

    max_right_bot_contour = sortContourShapes(right_contours_bot)[0]
    right_area_bot = cv2.contourArea(max_right_bot_contour) if is_valid_contour(max_right_bot_contour) else 0

    right_area = right_area_bot + right_area_top
    left_area = left_area_bot + left_area_top


    max_blue_contour1 = sortContourShapes(contours_blue1)[0] if contours_blue1 else None
    max_blue_area1 = cv2.contourArea(max_blue_contour1) if is_valid_contour(max_blue_contour1) else 0
    max_orange_contour1 = sortContourShapes(contours_orange1)[0] if contours_orange1 else None
    max_orange_area1 = cv2.contourArea(max_orange_contour1) if is_valid_contour(max_orange_contour1) else 0
    max_blue_contour2 = sortContourShapes(contours_blue2)[0] if contours_blue2 else None
    max_blue_area2 = cv2.contourArea(max_blue_contour2) if is_valid_contour(max_blue_contour2) else 0
    max_orange_contour2 = sortContourShapes(contours_orange2)[0] if contours_orange2 else None
    max_orange_area2 = cv2.contourArea(max_orange_contour2) if is_valid_contour(max_orange_contour2) else 0

    max_blue_area = max_blue_area2+max_blue_area1
    max_orange_area = max_orange_area2+max_orange_area1

    

    red_contours =  sortContourShapes(contours_red)
    max_red_contour = red_contours[0] if red_contours else None
    max_red_area = cv2.contourArea(max_red_contour) if is_valid_contour(max_red_contour) else 0

    green_contours = sortContourShapes(contours_green)
    max_green_contour = green_contours[0] if green_contours else None
    max_green_area = cv2.contourArea(max_green_contour) if is_valid_contour(max_green_contour) else 0

    red_contour2 = red_contours[1] if len(red_contours) > 1 else None
    red_area2 = cv2.contourArea(red_contour2) if is_valid_contour(red_contour2) else 0
    green_contour2 = green_contours[1] if len(green_contours) > 1 else None
    green_area2 = cv2.contourArea(green_contour2) if is_valid_contour(green_contour2) else 0


    if debug:
        # Draw max orange contour (orange)
        drawContour(max_orange_contour1, ROI_LINE1, frame, color=(0, 165, 255), thickness=2)

        # Draw max blue contour (blue)
        drawContour(max_blue_contour1, ROI_LINE1, frame, color=(255, 0, 0), thickness=2)    

        drawContour(max_orange_contour2, ROI_LINE2, frame, color=(0, 165, 255), thickness=2)

        # Draw max blue contour (blue)
        drawContour(max_blue_contour2, ROI_LINE2, frame, color=(255, 0, 0), thickness=2)
    
        # Draw max black contours (magenta for visibility)
        drawContour(max_left_top_contour, ROI_LEFT_TOP, frame, color=(255, 0, 255), thickness=2)

        drawContour(max_right_top_contour, ROI_RIGHT_TOP, frame, color=(255, 0, 255), thickness=2)

        drawContour(max_left_bot_contour, ROI_LEFT_BOT, frame, color=(255, 0, 255), thickness=2)

        drawContour(max_right_bot_contour, ROI_RIGHT_BOT, frame, color=(255, 0, 255), thickness=2)

        drawContour(max_green_contour, ROI_PILLAR, frame, color=(0, 255, 0), thickness=2)

        drawContour(max_red_contour, ROI_PILLAR, frame, color=(0, 0, 255), thickness=2)

        # Draw all ROIs with logical color coding
        
        drawRect(ROI_LEFT_TOP, frame, color=(255, 0, 0), thickness=2)        # Blue - top left
        drawRect(ROI_RIGHT_TOP, frame, color=(0, 0, 255), thickness=2)       # Red - top right
        drawRect(ROI_LEFT_BOT, frame, color=(0, 255, 0), thickness=2)       # Green - bottom left
        drawRect(ROI_RIGHT_BOT, frame, color=(255, 255, 0), thickness=2)     # Cyan - bottom right
        drawRect(ROI_LINE1, frame, color=(0, 255, 255), thickness=2)     # Yellow - center/orientation line
        drawRect(ROI_LINE2, frame, color=(0, 255, 255), thickness=2)     # Yellow - center/orientation line
        drawRect(ROI_PILLAR, frame, color=(0, 255, 255), thickness=2)


    size = 0

    if is_valid_contour(max_red_contour) and is_valid_contour(max_green_contour):
        if PILLAR_THRESH < cv2.contourArea(max_red_contour) > cv2.contourArea(max_green_contour):
            selected_contour = max_red_contour
            target = RED_TARGET

            if debug:
                cv2.line(frame, (RED_TARGET, 0), (RED_TARGET, 480), (0, 0, 255), 2)
        elif PILLAR_THRESH < cv2.contourArea(max_green_contour):
            selected_contour = max_green_contour
            target = GREEN_TARGET
            if debug:
                cv2.line(frame, (GREEN_TARGET, 0), (GREEN_TARGET, 480), (0, 255, 0), 2)
        else:
            selected_contour = None

    elif is_valid_contour(max_red_contour) and PILLAR_THRESH < cv2.contourArea(max_red_contour):
        selected_contour = max_red_contour
        target = RED_TARGET
        if debug:
            cv2.line(frame, (RED_TARGET, 0), (RED_TARGET, 480), (0, 0, 255), 2)

    elif is_valid_contour(max_green_contour) and PILLAR_THRESH < cv2.contourArea(max_green_contour):
        selected_contour = max_green_contour
        target = GREEN_TARGET
        if debug:
            cv2.line(frame, (GREEN_TARGET, 0), (GREEN_TARGET, 480), (0, 255, 0), 2)

    else:
        selected_contour = None


    if is_valid_contour(selected_contour):
        last_diff = 0
        curr_diff = 0
        size = cv2.contourArea(selected_contour)
        x, y, w, h = cv2.boundingRect(selected_contour)
        cX = x + w // 2 #+ ROI_PILLAR[0]
        cY = (y + h // 2) #+ ROI_PILLAR[1]
        #cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        print(cX,cY)
        # Normalize y-distance
        pillar_roi_bottom = ROI_PILLAR[3]
        pillar_roi_top = ROI_PILLAR[1]
        normalized_y = (cY - pillar_roi_top) / (pillar_roi_bottom - pillar_roi_top)
        normalized_y = np.clip(normalized_y, 0.0, 1.0)

        # Dynamic gain
        dynamic_gain = PILLAR_PG + (1 - normalized_y) * 0.0005

        if normalized_y < 0.2:
            RED_TARGET = 320
            GREEN_TARGET = 320
        elif normalized_y < 0.35:
            RED_TARGET = 160
            GREEN_TARGET = 460
        elif normalized_y < 0.4:
            RED_TARGET = 180
            GREEN_TARGET = 460
        elif normalized_y < 0.45:
            RED_TARGET = 150
            GREEN_TARGET = 460
        elif normalized_y < 0.60:
            RED_TARGET = 120
            GREEN_TARGET = 490
        elif normalized_y < 0.7:
            RED_TARGET = 100
            GREEN_TARGET = 530
        elif normalized_y < 0.85:
            RED_TARGET = 80
            GREEN_TARGET = 560
        else:
            RED_TARGET = 100
            GREEN_TARGET = 540

        error = cX-target
        
        angle = -(int((error * dynamic_gain) + ((error - last_pil_diff) * PILLAR_PD)))#*(normalized_y+1))#*(normalized_y/2+0.5)))
        
        
        print("target:", target, "left area:", left_area, "right area", right_area, "track_dir:", track_dir)

        if (cX  < 40 or cX > 600 ) and cY > 345 and not turning:
            angle = 0
            print("ignoring pillar")
            print(cX, cY)
        if (cX  < 100 or cX > 540 ) and cY > 325 and (red_area2 < PILLAR_THRESH and green_area2 < PILLAR_THRESH) and time_after_turn >= 30:
            angle = 0
            print("ignoring pillar EBCAUSW IR SAEEES NO NEXT")
            print(cX, cY)
    
        cv2.circle(frame, (cX, cY), 5, (255, 255, 255), -1)
        print("normalized_y: ", normalized_y)


    else:
        curr_diff = right_area - left_area
        angle = int((curr_diff * PG + (curr_diff-last_diff) * PD))
        print("using wallsISNDSNKSDNSJSJDJSJJSDJSD", angle)
        print(curr_diff, last_diff, curr_diff-last_diff)

        if turning:
            if track_dir==1:

                angle = -MAX_TURN_LESS
            else:
                angle = MAX_TURN_LESS

    if turning:
        print("TURFNDINFJDBNFDNNFDDJNFFNFE")
        PILLAR_THRESH = 150
        if track_dir == 1 and max_blue_area >= LINE_THRESH:
            turn_count += 1
            turning = False
            PILLAR_THRESH = 1200
            time_after_turn = 0
            print("TURN ENDED")
        elif track_dir == -1 and max_orange_area >= LINE_THRESH:
            turn_count += 1
            turning = False
            PILLAR_THRESH = 1200
            print("TURNENEDEDED")
            time_after_turn = 0

    elif track_dir == 0:
        if max_orange_area >= LINE_THRESH:
            track_dir = 1
            ROI_LINE1 = [0,0,0,0]
            ROI_LINE2 = [40,420,115,445]
            direction = "left"
        elif max_blue_area >= LINE_THRESH:
            track_dir = -1
            ROI_LINE2 = [0,0,0,0]
            ROI_LINE1 = [525,420,600,445]
            direction = "right"
            ATC = 29
    elif track_dir == 1:
        if max_orange_area >= LINE_THRESH:
            turning = True
            if NEXT_TURN:
                board.pwm_servo_set_position(0.1, [[1, pwm(MID_SERVO)]])
                board.pwm_servo_set_position(0.1, [[2, 1500]])
                break

        elif max_blue_area >= LINE_THRESH:
            pass

    elif track_dir == -1:
        if max_orange_area >= LINE_THRESH:
            pass
        elif max_blue_area >= LINE_THRESH:
            turning = True

    time_after_turn += 1
    if debug:
        cv2.imshow("Region of Interest", frame)
    cv2.waitKey(1)

    print(angle, "turn: ", turn_count)

    if angle>40:
        angle=40
    elif angle<-40:
        angle=-40

    servo = angle
    print(angle,speed)
    dc = speed
    board.pwm_servo_set_position(0.1, [[1, pwm(angle+MID_SERVO)]])
    board.pwm_servo_set_position(0.1, [[2, dc]])

    last_diff = curr_diff
    last_pill_diff= error
    #print(time.time()-s)
    
board.pwm_servo_set_position(0.1, [[1, pwm(MID_SERVO)]])
board.pwm_servo_set_position(0.1, [[2, 1500]])
LED(board,(0,0,0))
#lidar_power.off()
initial_yaw_deg = None
state = "cd"
error_sum = 0
last_error = 0
Kp, Ki, Kd = 0.3, 0.0, 0.1
initial_yaw_deg = None
offset_deg = 0
passed = False
angle = MID_SERVO
wall = False
start = True
first_time = 0
count = 0
flag = False
wait_til = None
if direction == "right":
    base_side_angle = 0.0
    state = "follow_wall"
    #board.pwm_servo_set_position(0.1, [[1, pwm(MID_SERVO)]])
    #board.pwm_servo_set_position(0.1, [[2, 1380]])
    time.sleep(1)
    board.pwm_servo_set_position(0.1, [[2, 1500]])
    initial_yaw_deg = get_yaw_deg()
    wait_til = 10
else:
    base_side_angle = 0.0
    state = "ss"
    initial_yaw_deg = get_yaw_deg()
    wait_til = 75
while not button_pressed:
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
    print(offset_deg)
    if state == "follow_wall":
        side = get_latest_distance((base_side_angle + offset_deg+360)%360)
        front = get_latest_distance(270.0)
    elif state == "turn_in":
        side = get_latest_distance((base_side_angle + offset_deg+360)%360)
        front = get_latest_distance((270.0 + offset_deg+360)%360)
    else:
        side = get_latest_distance(base_side_angle)
        front = get_latest_distance(270.0)
        
    print(state, offset_deg, front["distance"],side["distance"])
    if state == "ss":
        if front and front["distance"] <= 600:
            board.pwm_servo_set_position(0.1, [[1, pwm(MID_SERVO)]])
            board.pwm_servo_set_position(0.1, [[2, 1500]])
            state = "cd"
        else:
            board.pwm_servo_set_position(0.1, [[1, pwm(MID_SERVO)]])
            board.pwm_servo_set_position(0.1, [[2, 1605]])
    
    elif state == "cd":
        if not start:
            board.pwm_servo_set_position(0.1, [[1, pwm(MID_SERVO+MAX_TURN_DEGREE)]])
            board.pwm_servo_set_position(0.1, [[2, 1605]])
            if abs(offset_deg) >= 15:
                start = True
    
        
        if abs(offset_deg) >= 170 and start:
            board.pwm_servo_set_position(0.1, [[1, pwm(MID_SERVO)]])
            board.pwm_servo_set_position(0.1, [[2, 1500]])
            state = "follow_wall"
            
            initial_yaw_deg = get_yaw_deg()
            offset_deg = get_yaw_deg()-initial_yaw_deg
            side = get_latest_distance((base_side_angle + offset_deg+360)%360)
            print(side["distance"])
            time.sleep(1)
        
        else:
            board.pwm_servo_set_position(0.1, [[1, pwm(MID_SERVO+MAX_TURN_DEGREE)]])
            board.pwm_servo_set_position(0.1, [[2, 1605]])
            
    elif state == "follow_wall":
        print(first_time)
        #print(angle)
    
        error = TARGET_DIST - side["distance"]
        error_sum += error  # Just sum per iteration (integral)
        d_error = error - last_error  # Difference per iteration (derivative)

        pid_output = int(Kp * error + Ki * error_sum + Kd * d_error)
        last_error = error
        side2 = min([get_latest_distance(0.0),get_latest_distance((base_side_angle + offset_deg+360)%360),get_latest_distance(15.0),get_latest_distance(360-15)],key=lambda x:x["distance"])
        # Clamp steering angle
        print(side2["distance"])
        #front = get_latest_distance(270.0)
#             if right and 480 <= right["distance"] <= 520:
#                 Kp, Ki, Kd = 0.15, 0.0, 0.05
        if side["distance"] >= 455:
             first_time += 1
        if side2["distance"] <= 300 and first_time >= wait_til:
            board.pwm_servo_set_position(0.1, [[1, pwm(MID_SERVO)]])
            time.sleep(0.8)
            board.pwm_servo_set_position(0.1, [[2, 1500]])
            time.sleep(1)
            state = "turn_90"
            initial_yaw_deg = yaw_deg
            continue
            flag = True
            angle = 0


        else:
            if flag:
                state = "turn_90"
                board.pwm_servo_set_position(0.1, [[2, 1500]])
                board.pwm_servo_set_position(0.1, [[1, pwm(MID_SERVO)]])
                time.sleep(1)
                angle = 0
                #print(front["distance"])
                #break
                #continue
            
            else:
                angle = max(-20, min(20, pid_output))
        # Actuate motors
        board.pwm_servo_set_position(0.1, [[1, pwm(MID_SERVO + angle)]])
        board.pwm_servo_set_position(0.1, [[2, 1603]])  # DC motor
        #print()
    elif state == "turn_90":
        side = min([get_latest_distance(315.0),get_latest_distance(270.0),get_latest_distance(0.0)],key=lambda x:x["distance"])
        print(side["distance"])
        if abs(offset_deg) >= 70:
            state = "straight"
            board.pwm_servo_set_position(0.1, [[1, pwm(MID_SERVO)]])
            board.pwm_servo_set_position(0.1, [[2, 1500]])  # DC motor
            
#         elif side["distance"] <= 120:
#             board.pwm_servo_set_position(0.1, [[1, pwm(MID_SERVO-MAX_TURN_DEGREE)]])
#             board.pwm_servo_set_position(0.1, [[2, 1390]])  # DC motor
#             time.sleep(0.8)
#             board.pwm_servo_set_position(0.1, [[1, pwm(MID_SERVO)]])
#             board.pwm_servo_set_position(0.1, [[2, 1590]])  # DC motor
#             time.sleep(0.8)
        
        else:
            board.pwm_servo_set_position(0.1, [[1, pwm(MID_SERVO-MAX_TURN_DEGREE)]])
            board.pwm_servo_set_position(0.1, [[2, 1595]])  # DC motor
    elif state == "straight":
        front = get_latest_distance(270.0)
        print(front["distance"])
        if front and front["distance"] <= 130:
            state = "turn_in"
            board.pwm_servo_set_position(0.1, [[1, pwm(MID_SERVO)]])
            board.pwm_servo_set_position(0.1, [[2, 1500]])  # DC motor
            break
        else:
            board.pwm_servo_set_position(0.1, [[1, pwm(MID_SERVO+5)]])
            board.pwm_servo_set_position(0.1, [[2, 1595]])  # DC motor
        
    time.sleep(0.01)
        
board.pwm_servo_set_position(0.1, [[1, pwm(MID_SERVO)]])
board.pwm_servo_set_position(0.1, [[2, 1500]])
lidar_power.off()
LED(board,(0,0,0))
"""
print("lidar_thread alive?", any(t.name=="Thread-Name" and t.is_alive() for t in threading.enumerate()))

direction = "left"
base_side_angle = 180.0
sign = -1
error_sum = 0
last_error = 0
Kp, Ki, Kd = 0.3, 0.0, 0.1
state = "A"
initial_yaw_deg = None
offset_deg = None
while not button_pressed:
    
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
    
    if state == "detect_wall":
        front = get_latest_distance((270.0 + offset_deg + 360) % 360)
        left = get_latest_distance(180.0)
        right = get_latest_distance(0.0)
    elif state == "turn_out":
        front = get_latest_distance((270.0 + offset_deg + 360) % 360)
        side = get_latest_distance(base_side_angle)
        print(state,front["distance"], side["distance"], offset_deg)
    else:
        front = get_latest_distance(270.0)
        side = get_latest_distance((base_side_angle + offset_deg + 360) % 360)
        print(state,front["distance"], side["distance"], offset_deg)
        
        
    if not front or not side:
        continue
    
    if state == "A":
#         if front and front["distance"] < 500:
#             board.pwm_servo_set_position(0.1, [[1, pwm(MID_SERVO)]])
#             board.pwm_servo_set_position(0.1, [[2, 1500]])
#         
#         else:
            error = 450 - side["distance"]
            error_sum += error  # Just sum per iteration (integral)
            d_error = error - last_error  # Difference per iteration (derivative)

            pid_output = int(Kp * error + Ki * error_sum + Kd * d_error)
            last_error = error
            angle = max(-MAX_TURN_DEGREE, min(MAX_TURN_DEGREE, pid_output))
            print(angle)
            # Actuate motors
            board.pwm_servo_set_position(0.1, [[1, pwm(MID_SERVO - angle)]])
            board.pwm_servo_set_position(0.1, [[2, 1603]])  # DC motor


    
    time.sleep(0.01)
        

"""



