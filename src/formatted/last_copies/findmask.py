import cv2
import numpy as np
from picamera2 import Picamera2
from CONSTS import ideal_color_bgr_list

ideal_color_bgr = np.array(ideal_color_bgr_list, dtype=np.float32)  # target color
roi_selected = False
drawing = False
ix, iy, ex, ey = -1, -1, -1, -1

def draw_rectangle(event, x, y, flags, param):
    global ix, iy, ex, ey, drawing, roi_selected

    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        ix, iy = x, y
        roi_selected = False

    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing:
            ex, ey = x, y

    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False
        ex, ey = x, y
        roi_selected = True

def flatten_colors(img, K=8):
    Z = img.reshape((-1, 3))
    Z = np.float32(Z)

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
    _, labels, centers = cv2.kmeans(Z, K, None, criteria, 1, cv2.KMEANS_RANDOM_CENTERS)

    centers = np.uint8(centers)
    flattened = centers[labels.flatten()]
    return flattened.reshape(img.shape)

picam2 = Picamera2()
picam2.preview_configuration.main.size = (640,480)
picam2.preview_configuration.main.format = "RGB888"
print(picam2.preview_configuration.controls.FrameRate)
picam2.preview_configuration.controls.FrameRate = 30
picam2.set_controls({"Brightness": 1})
print(picam2.preview_configuration.controls.FrameRate)
picam2.preview_configuration.align()
picam2.configure("preview")

picam2.start()
shift = 0
cv2.namedWindow("Video")
cv2.setMouseCallback("Video", draw_rectangle)
while True:
    #ret, frame = cap.read()
    #if not ret:
    #break
    frame = picam2.capture_array()
    frame = cv2.resize(frame, (640, 480))
    display_frame = frame.copy()
    if drawing or roi_selected:
        cv2.rectangle(display_frame, (ix, iy), (ex, ey), (0, 255, 0), 2)

    corrected_frame = frame.copy()

    if roi_selected:
        x1, y1, x2, y2 = min(ix, ex), min(iy, ey), max(ix, ex), max(iy, ey)
        roi = frame[y1:y2, x1:x2]

        if roi.size > 0:
            observed_color_bgr = np.mean(roi.reshape(-1, 3), axis=0)
            observed_lab = cv2.cvtColor(np.uint8([[observed_color_bgr]]), cv2.COLOR_BGR2LAB)[0][0]
            ideal_lab = cv2.cvtColor(np.uint8([[ideal_color_bgr]]), cv2.COLOR_BGR2LAB)[0][0]

            shift = ideal_lab - observed_lab

            frame_lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB).astype(np.float32)
            frame_lab += shift
            frame_lab = np.clip(frame_lab, 0, 255).astype(np.uint8)
            frame_lab = cv2.GaussianBlur(frame_lab, (7, 7), 0)

            corrected_frame = cv2.cvtColor(frame_lab, cv2.COLOR_LAB2BGR)

            print("Observed LAB:", observed_lab)
            print("Ideal LAB:", ideal_lab)
            print("Shift:", shift)

    K = 8
    #if K < 2:  # minimum 2 clusters
        #K = 2

    flattened_frame = flatten_colors(corrected_frame, K)

    cv2.imshow("Video", display_frame)
    cv2.imshow("Corrected", corrected_frame)
    cv2.imshow("Flattened", flattened_frame)

    key = cv2.waitKey(1) & 0xFF
    if key == 27:  # ESC to quit
        print("Shift:", shift)
        break

#cap.release()
cv2.destroyAllWindows()

