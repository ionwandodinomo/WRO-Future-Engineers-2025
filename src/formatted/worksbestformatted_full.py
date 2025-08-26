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
#from exit_parking import unpark
import time
board = rcc.Board()
lidar_power = DigitalOutputDevice(LIDAR_POWER_PIN)
lidar_power.on()
time.sleep(1)
print("LD19 powered ON via GPIO 17.")

#red is right
#green is left

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

picam2 = Picamera2()
picam2.preview_configuration.main.size = (640,480)
picam2.preview_configuration.main.format = "RGB888"
print(picam2.preview_configuration.controls.FrameRate)
picam2.preview_configuration.controls.FrameRate = 75
picam2.set_controls({"Brightness": 0.05})
print(picam2.preview_configuration.controls.FrameRate)
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

frame = picam2.capture_array()
frame = cv2.resize(frame, (640, 480))

img_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
img_thresh = cv2.inRange(img_hsv, LOWER_BLACK_THRESHOLD, UPPER_BLACK_THRESHOLD)

left_contours_top, hierarchy = findContours(img_thresh, ROI_LEFT_TOP)
right_contours_top, hierarchy = findContours(img_thresh, ROI_RIGHT_TOP)
left_contours_bot, hierarchy = findContours(img_thresh, ROI_LEFT_BOT)
right_contours_bot, hierarchy = findContours(img_thresh, ROI_RIGHT_BOT)

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


latest_points = {}  # angle (int) → (distance, confidence, timestamp)

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



if right_area > left_area and right_area > 2000:
    track_dir = -1
elif left_area > right_area and left_area > 2000:
    track_dir = 1
     
if track_dir != 0:
    # unparking code here
    #unpark(board,track_dir)
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    buffer = bytearray()

    imu_proc = start_imu_stream()
    in_vector_block = False
    last_yaw = None

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

            d = get_latest_distance(90.0)
            if d:
                print(f"Latest distance at 90°: {d['distance']} mm (conf: {d['confidence']})")

            line = read_imu_line(imu_proc)
            if line:
                if line.startswith("vector:"):
                    in_vector_block = True
                elif in_vector_block and line.startswith("z:"):
                    try:
                        z_rad = float(line.split(":", 1)[1].strip())
                        z_deg = math.degrees(z_rad)
                        last_yaw = (z_rad, z_deg)
                        print(f"IMU yaw = {z_rad:.3f} rad ({z_deg:.1f}°)")
                    except ValueError:
                        pass
                    in_vector_block = False

            # DO UNPARKING CODE HERE
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Stopping lidar + IMU loop…")
    finally:
        stop_imu_stream(imu_proc)
        ser.close()
        lidar_power.off()
        time.sleep(1)   



if check_node_status():
    print("ROS2 node detected")
    listen_to_button_events() 
    print("STARTING OBSTACLE CHALLENGE CODE")



while True:
    frame = picam2.capture_array()
    frame = cv2.resize(frame, (640, 480))
    
    if turn_count >= 12:
        board.pwm_servo_set_position(0.1, [[1, pwm(MID_SERVO)]])
        board.pwm_servo_set_position(0.1, [[2, 1500]])
        time.sleep(0.1)
        break


    img_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    """
    img_lab = cv2.cvtColor(img, cv2.COLOR_BGR2Lab)
    img_lab = cv2.GaussianBlur(img_lab, (7, 7), 0)
    """

    img_thresh = cv2.inRange(img_hsv, LOWER_BLACK_THRESHOLD, UPPER_BLACK_THRESHOLD)
    img_thresh = cv2.bitwise_or(cv2.inRange(img_hsv, LOWER_MAGENTA_THRESHOLD, UPPER_MAGENTA_THRESHOLD),img_thresh)
    b_mask = cv2.inRange(img_hsv, LOWER_BLUE_THRESHOLD, UPPER_BLUE_THRESHOLD)
    o_mask = cv2.inRange(img_hsv, LOWER_ORANGE_THRESHOLD, UPPER_ORANGE_THRESHOLD)

    r_mask = cv2.bitwise_or(
            cv2.inRange(img_hsv, LOWER_RED_THRESHOLD1, UPPER_RED_THRESHOLD1),
            cv2.inRange(img_hsv, LOWER_RED_THRESHOLD2, UPPER_RED_THRESHOLD2),
        )
    g_mask = cv2.inRange(img_hsv, LOWER_GREEN_THRESHOLD, UPPER_GREEN_THRESHOLD)

    left_contours_top, hierarchy = findContours(img_thresh, ROI_LEFT_TOP)
    right_contours_top, hierarchy = findContours(img_thresh, ROI_RIGHT_TOP)
    left_contours_bot, hierarchy = findContours(img_thresh, ROI_LEFT_BOT)
    right_contours_bot, hierarchy = findContours(img_thresh, ROI_RIGHT_BOT)

    contours_blue1, _ = findContours(b_mask,ROI_LINE1)
    contours_orange1, _ = findContours(o_mask,ROI_LINE1)

    contours_blue2, _ = findContours(b_mask,ROI_LINE2)
    contours_orange2, _ = findContours(o_mask,ROI_LINE2)
    contours_red, _ = findContours(r_mask,ROI_PILLAR)
    contours_green = findContours(g_mask,ROI_PILLAR)

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


    if True:
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

    if is_valid_contour(max_red_contour) and is_valid_contour(max_green_contour):
        if PILLAR_THRESH < cv2.contourArea(max_red_contour) > cv2.contourArea(max_green_contour):
            selected_contour = max_red_contour
            target = RED_TARGET

            cv2.line(frame, (RED_TARGET, 0), (RED_TARGET, 480), (0, 0, 255), 2)
        elif PILLAR_THRESH < cv2.contourArea(max_green_contour):
            selected_contour = max_green_contour
            target = GREEN_TARGET
            cv2.line(frame, (GREEN_TARGET, 0), (GREEN_TARGET, 480), (0, 255, 0), 2)
        else:
            selected_contour = None

    elif is_valid_contour(max_red_contour) and PILLAR_THRESH < cv2.contourArea(max_red_contour):
        selected_contour = max_red_contour
        target = RED_TARGET
        cv2.line(frame, (RED_TARGET, 0), (RED_TARGET, 480), (0, 0, 255), 2)

    elif is_valid_contour(max_green_contour) and PILLAR_THRESH < cv2.contourArea(max_green_contour):
        selected_contour = max_green_contour
        target = GREEN_TARGET
        cv2.line(frame, (GREEN_TARGET, 0), (GREEN_TARGET, 480), (0, 255, 0), 2)

    else:
        selected_contour = None

    if is_valid_contour(selected_contour):
        last_diff = 0
        curr_diff = 0
        size = cv2.contourArea(selected_contour)
        x, y, w, h = cv2.boundingRect(selected_contour)
        cX = x + w // 2
        cY = y + h // 2

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
            RED_TARGET = 180
            GREEN_TARGET = 460
        elif normalized_y < 0.4:
            RED_TARGET = 135
            GREEN_TARGET = 405
        elif normalized_y < 0.45:
            RED_TARGET = 90
            GREEN_TARGET = 550
        else:
            RED_TARGET = 75
            GREEN_TARGET = 565
        
        
        error = cX-target
        
        angle = -(int((error * dynamic_gain) + ((error - last_pil_diff) * PILLAR_PD)))#*(normalized_y+1))#*(normalized_y/2+0.5)))
        
        
        print("target:", target, "left area:", left_area, "right area", right_area, "track_dir:", track_dir)

        if (cX  < 40 or cX > 600 ) and cY > 340:
            angle = 0
            print("ignoring pillar")
            print(cX, cY)
        if (cX  < 100 or cX > 540 ) and cY > 310 and (red_area2 < PILLAR_THRESH and green_area2 < PILLAR_THRESH) and not turning and time_after_turn >= 5:
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
        elif max_blue_area >= LINE_THRESH:
            track_dir = -1
            ROI_LINE2 = [0,0,0,0]
            ROI_LINE1 = [525,420,600,445]
            
    elif track_dir == 1:
        if max_orange_area >= LINE_THRESH:
            turning = True

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





# START PARKING CODE HERE SIMILAR TO LIDAR ABOVE