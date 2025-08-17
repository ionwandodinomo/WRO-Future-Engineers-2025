import cv2 as cv
from picamera2 import Picamera2
import numpy as np

LOWER_BLACK_THRESHOLD = np.array([0, 0, 0])
UPPER_BLACK_THRESHOLD = np.array([180, 255, 83])
LOWER_BLUE = np.array([91, 115, 103])
UPPER_BLUE = np.array([132, 255, 255])
LOWER_ORANGE1 = np.array([0, 101, 173])
UPPER_ORANGE1 = np.array([27, 255, 255])
LOWER_RED_THRESHOLD1 = np.array([0, 120, 138])
UPPER_RED_THRESHOLD1 = np.array([8, 255, 255])
LOWER_GREEN_THRESHOLD = np.array([71, 82, 75])
UPPER_GREEN_THRESHOLD = np.array([100, 255, 178])
LOWER_MAGENTA = np.array([140, 100, 100])
UPPER_MAGENTA = np.array([160, 255, 255])

max_value = 255
max_value_H = 180
low_H = 0
low_S = 0
low_V = 0
high_H = max_value_H
high_S = max_value
high_V = max_value

window_capture_name = 'Video Capture'
window_detection_name = 'Object Detection'
low_H_name = 'Low H'
low_S_name = 'Low S'
low_V_name = 'Low V'
high_H_name = 'High H'
high_S_name = 'High S'
high_V_name = 'High V'

def on_low_H_thresh_trackbar(val):
    global low_H, high_H
    low_H = min(val, high_H-1)
    cv.setTrackbarPos(low_H_name, window_detection_name, low_H)

def on_high_H_thresh_trackbar(val):
    global low_H, high_H
    high_H = max(val, low_H+1)
    cv.setTrackbarPos(high_H_name, window_detection_name, high_H)

def on_low_S_thresh_trackbar(val):
    global low_S, high_S
    low_S = min(val, high_S-1)
    cv.setTrackbarPos(low_S_name, window_detection_name, low_S)

def on_high_S_thresh_trackbar(val):
    global low_S, high_S
    high_S = max(val, low_S+1)
    cv.setTrackbarPos(high_S_name, window_detection_name, high_S)

def on_low_V_thresh_trackbar(val):
    global low_V, high_V
    low_V = min(val, high_V-1)
    cv.setTrackbarPos(low_V_name, window_detection_name, low_V)

def on_high_V_thresh_trackbar(val):
    global low_V, high_V
    high_V = max(val, low_V+1)
    cv.setTrackbarPos(high_V_name, window_detection_name, high_V)

picam2 = Picamera2()
picam2.preview_configuration.main.size = (640,480)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.controls.FrameRate = 25
picam2.set_controls({"Brightness": 0.05})
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

cv.namedWindow(window_capture_name)
cv.namedWindow(window_detection_name)

cv.createTrackbar(low_H_name, window_detection_name , low_H, max_value_H, on_low_H_thresh_trackbar)
cv.createTrackbar(high_H_name, window_detection_name , high_H, max_value_H, on_high_H_thresh_trackbar)
cv.createTrackbar(low_S_name, window_detection_name , low_S, max_value, on_low_S_thresh_trackbar)
cv.createTrackbar(high_S_name, window_detection_name , high_S, max_value, on_high_S_thresh_trackbar)
cv.createTrackbar(low_V_name, window_detection_name , low_V, max_value, on_low_V_thresh_trackbar)
cv.createTrackbar(high_V_name, window_detection_name , high_V, max_value, on_high_V_thresh_trackbar)

buttons = {
    "Black": {"pos": (10, 10, 100, 40), "lower": LOWER_BLACK_THRESHOLD, "upper": UPPER_BLACK_THRESHOLD},
    "Blue":  {"pos": (10, 60, 100, 40), "lower": LOWER_BLUE, "upper": UPPER_BLUE},
    "Orange": {"pos": (10, 110, 100, 40), "lower": LOWER_ORANGE1, "upper": UPPER_ORANGE1},
    "Red":   {"pos": (10, 160, 100, 40), "lower": LOWER_RED_THRESHOLD1, "upper": UPPER_RED_THRESHOLD1},
    "Green": {"pos": (10, 210, 100, 40), "lower": LOWER_GREEN_THRESHOLD, "upper": UPPER_GREEN_THRESHOLD}
    
}

def draw_buttons(img):
    for name, info in buttons.items():
        x, y, w, h = info["pos"]
        cv.rectangle(img, (x, y), (x+w, y+h), (200, 200, 200), -1)
        cv.putText(img, name, (x+5, y+25), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 2)

def mouse_callback(event, x, y, flags, param):
    global low_H, low_S, low_V, high_H, high_S, high_V
    if event == cv.EVENT_LBUTTONDOWN:
        for info in buttons.values():
            bx, by, bw, bh = info["pos"]
            if bx <= x <= bx+bw and by <= y <= by+bh:
                low_H, low_S, low_V = np.array(info["lower"])
                high_H, high_S, high_V = np.array(info["upper"])
                cv.setTrackbarPos(low_H_name, window_detection_name, low_H)
                cv.setTrackbarPos(high_H_name, window_detection_name, high_H)
                cv.setTrackbarPos(low_S_name, window_detection_name, low_S)
                cv.setTrackbarPos(high_S_name, window_detection_name, high_S)
                cv.setTrackbarPos(low_V_name, window_detection_name, low_V)
                cv.setTrackbarPos(high_V_name, window_detection_name, high_V)

cv.namedWindow("Buttons")
cv.setMouseCallback("Buttons", mouse_callback)

while True:
    frame = picam2.capture_array()
    frame_HSV = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    frame_threshold = cv.inRange(
        frame_HSV,
        np.array([low_H, low_S, low_V], dtype=np.uint8),
        np.array([high_H, high_S, high_V], dtype=np.uint8)
    )
    cv.imshow(window_capture_name, frame)
    cv.imshow(window_detection_name, frame_threshold)
    button_canvas = np.zeros((300, 150, 3), dtype=np.uint8)
    draw_buttons(button_canvas)
    cv.imshow("Buttons", button_canvas)
    key = cv.waitKey(30)
    if key == ord('q') or key == 27:
        break

cv.destroyAllWindows()




import cv2 as cv
from picamera2 import Picamera2
import numpy as np

LOWER_BLACK_THRESHOLD = np.array([0, 0, 0])
UPPER_BLACK_THRESHOLD = np.array([180, 255, 83])
LOWER_BLUE = np.array([91, 115, 103])
UPPER_BLUE = np.array([132, 255, 255])
LOWER_ORANGE1 = np.array([0, 101, 173])
UPPER_ORANGE1 = np.array([27, 255, 255])
LOWER_RED_THRESHOLD1 = np.array([0, 120, 138])
UPPER_RED_THRESHOLD1 = np.array([8, 255, 255])
LOWER_GREEN_THRESHOLD = np.array([71, 82, 75])
UPPER_GREEN_THRESHOLD = np.array([100, 255, 178])

max_value = 255
max_value_H = 180

window_capture_name = 'Video Capture'
window_detection_name = 'Object Detection'

low_H_name = 'Low H'
low_S_name = 'Low S'
low_V_name = 'Low V'
high_H_name = 'High H'
high_S_name = 'High S'
high_V_name = 'High V'

low_H = 0
low_S = 0
low_V = 0
high_H = max_value_H
high_S = max_value
high_V = max_value

def on_low_H_thresh_trackbar(val):
    global low_H, high_H
    low_H = min(val, high_H-1)
    cv.setTrackbarPos(low_H_name, window_detection_name, low_H)

def on_high_H_thresh_trackbar(val):
    global low_H, high_H
    high_H = max(val, low_H+1)
    cv.setTrackbarPos(high_H_name, window_detection_name, high_H)

def on_low_S_thresh_trackbar(val):
    global low_S, high_S
    low_S = min(val, high_S-1)
    cv.setTrackbarPos(low_S_name, window_detection_name, low_S)

def on_high_S_thresh_trackbar(val):
    global low_S, high_S
    high_S = max(val, low_S+1)
    cv.setTrackbarPos(high_S_name, window_detection_name, high_S)

def on_low_V_thresh_trackbar(val):
    global low_V, high_V
    low_V = min(val, high_V-1)
    cv.setTrackbarPos(low_V_name, window_detection_name, low_V)

def on_high_V_thresh_trackbar(val):
    global low_V, high_V
    high_V = max(val, low_V+1)
    cv.setTrackbarPos(high_V_name, window_detection_name, high_V)

picam2 = Picamera2()
picam2.preview_configuration.main.size = (640,480)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.controls.FrameRate = 25
picam2.set_controls({"Brightness": 0.05})
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

cv.namedWindow(window_capture_name)
cv.namedWindow(window_detection_name)

cv.createTrackbar(low_H_name, window_detection_name , low_H, max_value_H, on_low_H_thresh_trackbar)
cv.createTrackbar(high_H_name, window_detection_name , high_H, max_value_H, on_high_H_thresh_trackbar)
cv.createTrackbar(low_S_name, window_detection_name , low_S, max_value, on_low_S_thresh_trackbar)
cv.createTrackbar(high_S_name, window_detection_name , high_S, max_value, on_high_S_thresh_trackbar)
cv.createTrackbar(low_V_name, window_detection_name , low_V, max_value, on_low_V_thresh_trackbar)
cv.createTrackbar(high_V_name, window_detection_name , high_V, max_value, on_high_V_thresh_trackbar)

buttons = {
    "Black": {"pos": (10, 10, 100, 40), "lower": LOWER_BLACK_THRESHOLD, "upper": UPPER_BLACK_THRESHOLD},
    "Blue":  {"pos": (10, 60, 100, 40), "lower": LOWER_BLUE, "upper": UPPER_BLUE},
    "Orange": {"pos": (10, 110, 100, 40), "lower": LOWER_ORANGE1, "upper": UPPER_ORANGE1},
    "Red":   {"pos": (10, 160, 100, 40), "lower": LOWER_RED_THRESHOLD1, "upper": UPPER_RED_THRESHOLD1},
    "Green": {"pos": (10, 210, 100, 40), "lower": LOWER_GREEN_THRESHOLD, "upper": UPPER_GREEN_THRESHOLD},
    "Save": {"pos": (10, 260, 100, 40)}
}

selected_color = "Black"

color_thresholds = {
    "Black": [LOWER_BLACK_THRESHOLD.copy(), UPPER_BLACK_THRESHOLD.copy()],
    "Blue": [LOWER_BLUE.copy(), UPPER_BLUE.copy()],
    "Orange": [LOWER_ORANGE1.copy(), UPPER_ORANGE1.copy()],
    "Red": [LOWER_RED_THRESHOLD1.copy(), UPPER_RED_THRESHOLD1.copy()],
    "Green": [LOWER_GREEN_THRESHOLD.copy(), UPPER_GREEN_THRESHOLD.copy()]
}

def draw_buttons(img):
    for name, info in buttons.items():
        x, y, w, h = info["pos"]
        if name == selected_color:
            color_rect = (0, 255, 0)  # green highlight for selected
        else:
            color_rect = (200, 200, 200)
        cv.rectangle(img, (x, y), (x+w, y+h), color_rect, -1)
        cv.putText(img, name, (x+5, y+25), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 2)


def save_sliders_to_color():
    lower, upper = color_thresholds[selected_color]
    lower[0] = cv.getTrackbarPos('Low H', window_detection_name)
    lower[1] = cv.getTrackbarPos('Low S', window_detection_name)
    lower[2] = cv.getTrackbarPos('Low V', window_detection_name)
    upper[0] = cv.getTrackbarPos('High H', window_detection_name)
    upper[1] = cv.getTrackbarPos('High S', window_detection_name)
    upper[2] = cv.getTrackbarPos('High V', window_detection_name)

    for name, (low, up) in color_thresholds.items():
        var_low = f"LOWER_{name.upper().replace(' ', '_')}_THRESHOLD" if name != "Orange" else "LOWER_ORANGE1"
        var_up = f"UPPER_{name.upper().replace(' ', '_')}_THRESHOLD" if name != "Orange" else "UPPER_ORANGE1"
        print(f"{var_low} = np.array([{low[0]}, {low[1]}, {low[2]}])")
        print(f"{var_up} = np.array([{up[0]}, {up[1]}, {up[2]}])")
    print("\n")

def mouse_callback(event, x, y, flags, param):
    global selected_color
    if event == cv.EVENT_LBUTTONDOWN:
        for name, info in buttons.items():
            bx, by, bw, bh = info["pos"]
            if bx <= x <= bx+bw and by <= y <= by+bh:
                if name == "Save":
                    save_sliders_to_color()
                else:
                    selected_color = name
                    lower, upper = color_thresholds[selected_color]
                    cv.setTrackbarPos(low_H_name, window_detection_name, lower[0])
                    cv.setTrackbarPos(high_H_name, window_detection_name, upper[0])
                    cv.setTrackbarPos(low_S_name, window_detection_name, lower[1])
                    cv.setTrackbarPos(high_S_name, window_detection_name, upper[1])
                    cv.setTrackbarPos(low_V_name, window_detection_name, lower[2])
                    cv.setTrackbarPos(high_V_name, window_detection_name, upper[2])

cv.namedWindow("Buttons")
cv.setMouseCallback("Buttons", mouse_callback)

while True:
    frame = picam2.capture_array()
    frame_HSV = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    lower, upper = color_thresholds[selected_color]
    frame_threshold = cv.inRange(frame_HSV, lower, upper)
    
    cv.imshow(window_capture_name, frame)
    cv.imshow(window_detection_name, frame_threshold)

    button_canvas = np.zeros((320, 150, 3), dtype=np.uint8)
    draw_buttons(button_canvas)
    cv.imshow("Buttons", button_canvas)

    key = cv.waitKey(30)
    if key == ord('q') or key == 27:
        save_sliders_to_color()
        break

cv.destroyAllWindows()
