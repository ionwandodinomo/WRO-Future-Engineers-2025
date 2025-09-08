import cv2 as cv
from picamera2 import Picamera2
import numpy as np

LOWER_BLACK_THRESHOLD = np.array([0, 0, 0])
UPPER_BLACK_THRESHOLD = np.array([180, 255, 83])
LOWER_BLUE_THRESHOLD = np.array([91, 115, 103])
UPPER_BLUE_THRESHOLD = np.array([132, 255, 255])
LOWER_ORANGE_THRESHOLD = np.array([0, 101, 173])
UPPER_ORANGE_THRESHOLD = np.array([27, 255, 255])
LOWER_GREEN_THRESHOLD = np.array([71, 82, 75])
UPPER_GREEN_THRESHOLD = np.array([100, 255, 178])
LOWER_MAGENTA_THRESHOLD = np.array([164, 0, 0])
UPPER_MAGENTA_THRESHOLD = np.array([171, 255, 255])
LOWER_RED_THRESHOLD1 = np.array([0, 138, 138])
UPPER_RED_THRESHOLD1 = np.array([7, 255, 255])
LOWER_RED_THRESHOLD2 = np.array([171, 120, 136])
UPPER_RED_THRESHOLD2 = np.array([180, 255, 255])


picam2 = Picamera2()
picam2.preview_configuration.main.size = (640,480)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.controls.FrameRate = 25
picam2.set_controls({"Brightness": 0.05})
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

window_capture_name, window_detection_name = 'Video Capture', 'Object Detection'

def make_sliders(name,low,up):
    cv.createTrackbar(f'Low H{name}', window_detection_name , low[0], 180, lambda v:None)
    cv.createTrackbar(f'High H{name}', window_detection_name , up[0], 180, lambda v:None)
    cv.createTrackbar(f'Low S{name}', window_detection_name , low[1], 255, lambda v:None)
    cv.createTrackbar(f'High S{name}', window_detection_name , up[1], 255, lambda v:None)
    cv.createTrackbar(f'Low V{name}', window_detection_name , low[2], 255, lambda v:None)
    cv.createTrackbar(f'High V{name}', window_detection_name , up[2], 255, lambda v:None)

def read_sliders(name):
    return np.array([cv.getTrackbarPos(f'Low H{name}', window_detection_name),
                     cv.getTrackbarPos(f'Low S{name}', window_detection_name),
                     cv.getTrackbarPos(f'Low V{name}', window_detection_name)]), \
           np.array([cv.getTrackbarPos(f'High H{name}', window_detection_name),
                     cv.getTrackbarPos(f'High S{name}', window_detection_name),
                     cv.getTrackbarPos(f'High V{name}', window_detection_name)])

cv.namedWindow(window_capture_name); cv.namedWindow(window_detection_name)

buttons = {
    "Black":   {"pos": (10, 10, 100, 40), "lower": LOWER_BLACK_THRESHOLD, "upper": UPPER_BLACK_THRESHOLD},
    "Blue":    {"pos": (10, 60, 100, 40), "lower": LOWER_BLUE_THRESHOLD, "upper": UPPER_BLUE_THRESHOLD},
    "Orange":  {"pos": (10, 110, 100, 40), "lower": LOWER_ORANGE_THRESHOLD, "upper": UPPER_ORANGE_THRESHOLD},
    "Red":     {"pos": (10, 160, 100, 40)},
    "Green":   {"pos": (10, 210, 100, 40), "lower": LOWER_GREEN_THRESHOLD, "upper": UPPER_GREEN_THRESHOLD},
    "Magenta": {"pos": (10, 260, 100, 40), "lower": LOWER_MAGENTA_THRESHOLD, "upper": UPPER_MAGENTA_THRESHOLD},
    "Save":    {"pos": (10, 310, 100, 40)}
}
selected_color = "Black"


color_thresholds = {
    "Black": [LOWER_BLACK_THRESHOLD.copy(), UPPER_BLACK_THRESHOLD.copy()],
    "Blue": [LOWER_BLUE_THRESHOLD.copy(), UPPER_BLUE_THRESHOLD.copy()],
    "Orange": [LOWER_ORANGE_THRESHOLD.copy(), UPPER_ORANGE_THRESHOLD.copy()],
    "Green": [LOWER_GREEN_THRESHOLD.copy(), UPPER_GREEN_THRESHOLD.copy()],
    "Magenta": [LOWER_MAGENTA_THRESHOLD.copy(), UPPER_MAGENTA_THRESHOLD.copy()]
}

def draw_buttons(img):
    for name, info in buttons.items():
        x,y,w,h = info["pos"]
        color_rect = (0,255,0) if name == selected_color else (200,200,200)
        cv.rectangle(img, (x,y), (x+w,y+h), color_rect, -1)
        cv.putText(img, name, (x+5,y+25), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 2)

def save_sliders_to_color():
    global LOWER_RED_THRESHOLD1,UPPER_RED_THRESHOLD1,LOWER_RED_THRESHOLD2,UPPER_RED_THRESHOLD2
    if selected_color=="Red":
        LOWER_RED_THRESHOLD1,UPPER_RED_THRESHOLD1 = read_sliders("1")
        LOWER_RED_THRESHOLD2,UPPER_RED_THRESHOLD2 = read_sliders("2")
    else:
        lower, upper = color_thresholds[selected_color]
        lower[:], upper[:] = read_sliders("")
    for name,(low,up) in color_thresholds.items():
        var_low,var_up=f"LOWER_{name.upper()}_THRESHOLD",f"UPPER_{name.upper()}_THRESHOLD"
        print(f"{var_low} = np.array([{low[0]}, {low[1]}, {low[2]}])"); print(f"{var_up} = np.array([{up[0]}, {up[1]}, {up[2]}])")
    print(f"LOWER_RED_THRESHOLD1 = np.array([{LOWER_RED_THRESHOLD1[0]}, {LOWER_RED_THRESHOLD1[1]}, {LOWER_RED_THRESHOLD1[2]}])")
    print(f"UPPER_RED_THRESHOLD1 = np.array([{UPPER_RED_THRESHOLD1[0]}, {UPPER_RED_THRESHOLD1[1]}, {UPPER_RED_THRESHOLD1[2]}])")
    print(f"LOWER_RED_THRESHOLD2 = np.array([{LOWER_RED_THRESHOLD2[0]}, {LOWER_RED_THRESHOLD2[1]}, {LOWER_RED_THRESHOLD2[2]}])")
    print(f"UPPER_RED_THRESHOLD2 = np.array([{UPPER_RED_THRESHOLD2[0]}, {UPPER_RED_THRESHOLD2[1]}, {UPPER_RED_THRESHOLD2[2]}])\n")

def mouse_callback(event,x,y,flags,param):
    global selected_color
    if event==cv.EVENT_LBUTTONDOWN:
        for name,info in buttons.items():
            bx,by,bw,bh = info["pos"]
            if bx<=x<=bx+bw and by<=y<=by+bh:
                if name=="Save": save_sliders_to_color()
                else:
                    selected_color=name
                    if name=="Red":
                        make_sliders("1",LOWER_RED_THRESHOLD1,UPPER_RED_THRESHOLD1)
                        make_sliders("2",LOWER_RED_THRESHOLD2,UPPER_RED_THRESHOLD2)
                    else:
                        lower,upper=color_thresholds[selected_color]; make_sliders("",lower,upper)

cv.namedWindow("Buttons"); cv.setMouseCallback("Buttons", mouse_callback)

make_sliders("",color_thresholds[selected_color][0],color_thresholds[selected_color][1])

while True:
    frame = picam2.capture_array()
    frame_HSV = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    if selected_color=="Red":
        l1,u1=read_sliders("1"); l2,u2=read_sliders("2")
        mask1=cv.inRange(frame_HSV,l1,u1); mask2=cv.inRange(frame_HSV,l2,u2)
        frame_threshold=cv.bitwise_or(mask1,mask2)
    else:
        lower,upper=read_sliders(""); frame_threshold=cv.inRange(frame_HSV,lower,upper)
    cv.imshow(window_capture_name, frame); cv.imshow(window_detection_name, frame_threshold)
    button_canvas = np.zeros((360,150,3),dtype=np.uint8); draw_buttons(button_canvas); cv.imshow("Buttons", button_canvas)
    key = cv.waitKey(30)
    if key==ord('q') or key==27: save_sliders_to_color(); break

cv.destroyAllWindows()
color_thresholds = {
    "Black": [LOWER_BLACK_THRESHOLD.copy(), UPPER_BLACK_THRESHOLD.copy()],
    "Blue": [LOWER_BLUE_THRESHOLD.copy(), UPPER_BLUE_THRESHOLD.copy()],
    "Orange": [LOWER_ORANGE_THRESHOLD.copy(), UPPER_ORANGE_THRESHOLD.copy()],
    "Green": [LOWER_GREEN_THRESHOLD.copy(), UPPER_GREEN_THRESHOLD.copy()],
    "Magenta": [LOWER_MAGENTA_THRESHOLD.copy(), UPPER_MAGENTA_THRESHOLD.copy()]
}

def draw_buttons(img):
    for name, info in buttons.items():
        x,y,w,h = info["pos"]
        color_rect = (0,255,0) if name == selected_color else (200,200,200)
        cv.rectangle(img, (x,y), (x+w,y+h), color_rect, -1)
        cv.putText(img, name, (x+5,y+25), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 2)

def save_sliders_to_color():
    global LOWER_RED_THRESHOLD1,UPPER_RED_THRESHOLD1,LOWER_RED_THRESHOLD2,UPPER_RED_THRESHOLD2
    if selected_color=="Red":
        LOWER_RED_THRESHOLD1,UPPER_RED_THRESHOLD1 = read_sliders("1")
        LOWER_RED_THRESHOLD2,UPPER_RED_THRESHOLD2 = read_sliders("2")
    else:
        lower, upper = color_thresholds[selected_color]
        lower[:], upper[:] = read_sliders("")
    for name,(low,up) in color_thresholds.items():
        var_low,var_up=f"LOWER_{name.upper()}_THRESHOLD",f"UPPER_{name.upper()}_THRESHOLD"
        print(f"{var_low} = np.array([{low[0]}, {low[1]}, {low[2]}])"); print(f"{var_up} = np.array([{up[0]}, {up[1]}, {up[2]}])")
    print(f"LOWER_RED_THRESHOLD1 = np.array([{LOWER_RED_THRESHOLD1[0]}, {LOWER_RED_THRESHOLD1[1]}, {LOWER_RED_THRESHOLD1[2]}])")
    print(f"UPPER_RED_THRESHOLD1 = np.array([{UPPER_RED_THRESHOLD1[0]}, {UPPER_RED_THRESHOLD1[1]}, {UPPER_RED_THRESHOLD1[2]}])")
    print(f"LOWER_RED_THRESHOLD2 = np.array([{LOWER_RED_THRESHOLD2[0]}, {LOWER_RED_THRESHOLD2[1]}, {LOWER_RED_THRESHOLD2[2]}])")
    print(f"UPPER_RED_THRESHOLD2 = np.array([{UPPER_RED_THRESHOLD2[0]}, {UPPER_RED_THRESHOLD2[1]}, {UPPER_RED_THRESHOLD2[2]}])\n")

def mouse_callback(event,x,y,flags,param):
    global selected_color
    if event==cv.EVENT_LBUTTONDOWN:
        for name,info in buttons.items():
            bx,by,bw,bh = info["pos"]
            if bx<=x<=bx+bw and by<=y<=by+bh:
                if name=="Save": save_sliders_to_color()
                else:
                    selected_color=name
                    if name=="Red":
                        make_sliders("1",LOWER_RED_THRESHOLD1,UPPER_RED_THRESHOLD1)
                        make_sliders("2",LOWER_RED_THRESHOLD2,UPPER_RED_THRESHOLD2)
                    else:
                        lower,upper=color_thresholds[selected_color]; make_sliders("",lower,upper)

cv.namedWindow("Buttons"); cv.setMouseCallback("Buttons", mouse_callback)


while True:
    frame = picam2.capture_array()
    frame_HSV = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    if selected_color=="Red":
        l1,u1=read_sliders("1"); l2,u2=read_sliders("2")
        mask1=cv.inRange(frame_HSV,l1,u1); mask2=cv.inRange(frame_HSV,l2,u2)
        frame_threshold=cv.bitwise_or(mask1,mask2)
    else:
        lower,upper=read_sliders(""); frame_threshold=cv.inRange(frame_HSV,lower,upper)
    cv.imshow(window_capture_name, frame); cv.imshow(window_detection_name, frame_threshold)
    button_canvas = np.zeros((360,150,3),dtype=np.uint8); draw_buttons(button_canvas); cv.imshow("Buttons", button_canvas)
    key = cv.waitKey(30)
    if key==ord('q') or key==27: save_sliders_to_color(); break

cv.destroyAllWindows()

