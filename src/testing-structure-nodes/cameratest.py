import cv2
import numpy as np
from picamera2 import Picamera2

# Initialize camera
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640,480)
picam2.preview_configuration.main.format = "RGB888"
print(picam2.preview_configuration.controls.FrameRate)
picam2.preview_configuration.controls.FrameRate = 25
picam2.set_controls({"Brightness": 0.05})
print(picam2.preview_configuration.controls.FrameRate)
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()


# Define rectangular ROIs
left_roi = (0, 255, 130, 80)     # x, y, w, h
right_roi = (510, 255, 130, 80)
center_roi = (300,280, 50,25)

print("Running Region of Interest Viewer. Press 'q' to quit.")

while True:
    frame = picam2.capture_array()
    frame = cv2.resize(frame, (640, 480))

    # Draw left and right ROI rectangles
    cv2.rectangle(frame, (left_roi[0], left_roi[1]),
                  (left_roi[0] + left_roi[2], left_roi[1] + left_roi[3]), (255, 0, 0), 2)
    cv2.rectangle(frame, (right_roi[0], right_roi[1]),
                  (right_roi[0] + right_roi[2], right_roi[1] + right_roi[3]), (0, 0, 255), 2)
    
    cv2.rectangle(frame, (center_roi[0], center_roi[1]),
                  (center_roi[0] + center_roi[2], center_roi[1] + center_roi[3]), (0, 0, 255), 2)

    # Optional: Label ROIs
    cv2.putText(frame, "LEFT ROI", (left_roi[0], left_roi[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
    cv2.putText(frame, "RIGHT ROI", (right_roi[0], right_roi[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    
    cv2.putText(frame, "Line ROI", (center_roi[0], center_roi[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1) 

    # Show live view
    cv2.imshow("Region of Interest", frame)

    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()



