# constant values that are used in both obstacle and open challenge. this is used as an import file to keep things consistant accross all codes
import numpy as np

# colour management
rMagenta = [[0, 169, 110], [255, 255, 141]]
rRed = [[0, 157, 135], [179, 255, 227]]
rGreen = [[103, 0, 135], [235, 113, 178]]
rBlue = [[106, 0, 0], [255, 255, 114]]
rOrange = [[147, 148, 132], [255, 255, 192]]
rBlack = [[5, 0, 0], [78, 255, 255]]

ideal_color_bgr_list = [255,255,255]
SHIFT_FROM_MASK = [36,4,2]

# pins
LIDAR_POWER_PIN = 17

# rois
ROI_LEFT_TOP = [0, 220, 100, 270]
ROI_RIGHT_TOP = [540, 220, 640, 270]
ROI_LEFT_BOT = [0, 270, 40, 295]
ROI_RIGHT_BOT = [600, 270, 640, 295]

ROI_LINE1 = [277,250,352,275]
ROI_LINE2 = [0,0,0,0]
ROI_PILLAR = [0,150,640,380]

# default targets
RED_TARGET = 110
GREEN_TARGET = 530

# PID controls
PD = 0.0001
PG = 0.006
PDLOW = 0.00001
PGLOW = 0.0015
PILLAR_PD = 0.3
PILLAR_PG = 0.05

# threshold/clamps
LINE_THRESH = 25
MAX_TURN_DEGREE = 40
MAX_TURN_LESS = 20
PILLAR_THRESH = 1200
MID_SERVO = -3