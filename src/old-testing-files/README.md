Control software
====

This directory must contain code for control software which is used by the vehicle to participate in the competition and which was developed by the participants.

All artifacts required to resolve dependencies and build the project must be included in this directory as well.





# Control Software Documentation

## Introduction
This directory contains the control software for the competition vehicle.  
It runs entirely on a **Raspberry Pi 5** with a HiWonder motor hat, controlling **DC motors, ESC, and a steering servo**.  

- A **USB camera** provides real-time vision.  
- A **LiDAR** module is used exclusively for the parking sequence.  
- The vehicle maintains **constant motor speed**, while steering is controlled by a **servo with PID-based correction**.  

The software is written in **Python**.  
Earlier iterations used **ROS2**, but it was abandoned due to performance constraints: building from source on the Pi made it slower than a single-file Python solution. Since concurrency could be managed directly, ROS2 was not required.

---

## Hardware Overview

| Component        | Description                                         |
|------------------|-----------------------------------------------------|
| Raspberry Pi 5   | Main onboard controller                             |
| HiWonder Hat     | Provides PWM control for motors and servo           |
| DC Motor + ESC   | Constant propulsion                                |
| Servo Motor      | Steering (angle calculated via PID)                 |
| USB Camera       | Color-based image detection                         |
| LiDAR            | Used **only** during the parking sequence           |

![Hardware Overview](docs/hardware_overview.png)

---

## Software Architecture
The control software is structured into logical modules:

- **Image Processing**  
  - Color masking and contour detection (LAB color space).  
  - Extracts walls, pillars, lane markers, and parking zones.  

- **Navigation Control**  
  - PID-based steering for wall following or pillar following.  

- **Challenge Logic**  
  - Open Challenge: Wall following + turn counting.  
  - Obstacle Challenge: Parking, straight/turn logic, pillar handling.  

- **Failsafes**  
  - Timer-based sequences (e.g., fixed straight drive after last turn).  

All actuation is performed with direct **PWM control** through the HiWonder hat.

![Software Architecture](docs/software_architecture.png)

---

## Image Detection & Processing

The vehicle initially used **HSV color space**, but switched to **LAB** due to its better robustness under different lighting conditions.

### Processing Pipeline
1. Apply LAB color thresholds to generate binary masks.  
2. Apply **fixed ROIs** for walls, pillars, and lines.  
3. Extract contours with `cv2.findContours`.  
4. Select largest contour in each ROI for stability.  
5. Compute bounding boxes and centroids `(cX, cY)` for navigation logic.

![Image Processing Pipeline](docs/image_processing_pipeline.png)

### Color Mapping

| Object        | Color (in LAB mask)   |
|---------------|-----------------------|
| Walls         | Black                 |
| Lane Markers  | Blue & Orange         |
| Pillars       | Red & Green           |
| Parking Zone  | Magenta               |

---

## Open Challenge Strategy

The **Open Challenge** relies on **wall following** and **turn counting**.

![Open Challenge Flow](docs/open_challenge_flow.png)

### Wall Following
- **Wall difference** = pixel area difference between left and right wall ROIs.  
- Error = `(left_area - right_area)`.  
- PID control adjusts steering:  

```python
servo_angle = -(error * P_GAIN) - ((error - last_error) * D_GAIN)
```

- If both ROIs are heavily filled (walls close), PID gain is reduced to prevent oscillation.

### Turn Detection

- Track has physical blue/orange lines marking turns.
- Each line crossing increments a turn counter.
- After 12 turns, the vehicle drives straight for a fixed time to finish in the correct section.


## Obstacle Challenge Strategy
### Parking & Start
- Vehicle begins parked.
- Uses ROI wall detection (left vs right).
- If wall area > threshold, orientation is determined.
- Unparking sequence: Time-based, no feedback → ends aligned with track.

### Navigation
- The course alternates between straight sections and turning sections.

### Mode Switching
- Blue/Orange lines on track → entering a turn.
- Otherwise → straight section.

### Straight Sections
- If pillars exist:
    - Define target line = vertical line in image.
    - Error = target_x - pillar_cX.
    - Steering = PID(error).
    - Ignore pillar once cY > threshold (passed pillar).
- If no pillars:
    - Fall back to wall following (more sensitive PID than Open Challenge).
### Turning Sections:
- Pillar threshold is lowered by ~300% (detect further pillars).
- If pillars exist:
    - Prioritize pillar following.
    - Ignore pillars later than straight mode to complete tight turns.

- If no pillars:
    - Execute fixed-angle turn until second line marker is detected.
    - Parking Zone Handling
    - Parking areas (magenta) are ignored during normal navigation.
    - Implemented as: wall_mask = black_mask OR magenta_mask

## PID Control

Only Proportional (P) and Derivative (D) terms are used.
Motor speed is constant; only steering is adjusted.

- General Formula: `angle = -(int((error * dynamic_gain) + ((error - last_error) * PD_GAIN)))` (negative due to nature of servo)

Gains
Challenge	P Gain	D Gain
Open Challenge	0.0045	0.00004
Obstacle Challenge	More sensitive (narrower track margins)	

Software Setup Guide
Dependencies

- Python 3
- OpenCV (cv2)
- NumPy
- Time
- HiWonder hat `ros_robot_sdk.py` (LINK HERE)

- serial, RPi.GPIO (for LiDAR if enabled, obstacle challenge only)

Running the Code
- Connect Raspberry Pi 5 with camera & HiWonder hat
- connect to pi using AP or STA mode. run `sdskdfs.py`


