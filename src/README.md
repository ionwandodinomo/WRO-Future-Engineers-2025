Control software
====

This directory must contain code for control software which is used by the vehicle to participate in the competition and which was developed by the participants.

All artifacts required to resolve dependencies and build the project must be included in this directory as well.

DOCCUMENTATION WRITTEN TTENTSATIVELY HERE TO MITIGATE MERGE CONFLICTS. MOFVE TO REPO README.MD LATER

image detection and processing:
cv2.contours based on hsv originally. 
however chose to use lab for more consistancy
find connbtaours based on hsv range thresholds to create masks.
most colours only care about the largest contour in the roi


### Open Challenge
## Overview of Challenge
The open challenge is a time attack round, where each team has just 3 minutes to drive 3 laps autonomously on a track. Every round of the challenge, the layout is randomized, where each straight section can either consist of wide walls, leaving a large gap between the inside or outside walls, or narrow walls, giving the car much less space to manoeuvre. The track direction is also randomized. The goal of the open challenge is to finish the laps as fast and consistently as possible, not touching the inside walls or crashing the vehicle.

The difficulty of the challenge comes from the changes in the width of the track, where the track turns from narrow to wide or from wide to narrow. This drastic change in the track can confuse the car, creating instances where it is difficult to ascertain the correct direction of the track or for the car to recognise the walls.

ADD PHOTO OF MAP FROM BIRDS-EYE VIEW
<img src="/other/placeholder.png" height="250">

#### Our Solution
To break down the challenge into manageable parts, we thought of it as three parts. These three parts are comprised of turning, driving in the straights, and counting the turns. In all three parts, 5 regions of interest were used. 4 of these regions are tracking the walls, using a LAB or HSV range to check for the black colour of the walls, returning a number for the total number of pixels of the left two regions, and a number for the total number of pixels of the right two regions. The last region of interest is used to track the blue and orange lines on the track, allowing us to ascertain the correct direction and the corners of the track.

ADD SS OF DEBUG MODE DURING OPEN CHALLENGE
<img src="/other/placeholder.png" height="250">

##### Turning
The turning of the car is based on the threshold of the inside wall. Based on the direction of the track, when the number of pixels of a wall reaches below a certain threshold, we start the turn by locking the steering angle until the number of pixels in the inside wall climbs back above the threshold. This allows us to adjust during turns, signalling to start a turn when we detect that a wall disappears, usually indicating when the inside wall ends.
```python
if left_area < WALL_THRESH:
  angle = TURN_ANGLE
elif right_area < WALL_THRESH:
  angle = -TURN ANGLE
```
Past this initial turn, the rest of the turn relies on the angle calculating method in the straights.
##### Straights
Driving in the straights uses a PD calculating algorithm. This PD algorithm is comprised of proportional and derivative angle calculating, comprised of two constants: PD and PG. First, we find the difference in the number of pixels of the wall and multiply it by the PD, and add the variation of the last two differences multiplied by the PD. This allows the car to adjust the car to the walls of the track, making sure we don't veer into a crash, while adjusting based on the past patterns.
```python
angle = int((curr_diff * PG + (curr_diff-last_diff) * PD))
```
However, to make the car turn earlier, we had to increase the two constants to make the car more sensitive to changes. This caused oscillations in the straights due to the high sensitivity. To fix this issue, when we detect a large amount of pixels in both walls, we assume we are in a straight and use a lower PD and PG.
```python
if right_area > 1250 and left_area > 1250:
  angle = int((curr_diff * PGLOW + (curr_diff-last_diff) * PDLOW))
else:
  angle = int((curr_diff * PG + (curr_diff-last_diff) * PD))
```
##### Turn Counting
Turn counting is made simple by the lines on the track. When we detect the very first line, the colour of this line indicates the direction of the track. Starting from then, every time we see that first colour followed by the second colour, we add one to the turn count.
First, we find the direction of the track from the first line.
```python
if track_dir == 0:
  if max_orange_area >= LINE_THRESH:
  track_dir = 1
elif max_blue_area >= LINE_THRESH:
  track_dir = -1
```
For all following turns, we start the turn by detecting the line we first saw in the track.
```python
if track_dir == 1:
  if max_orange_area >= LINE_THRESH:
  turning = True

elif track_dir == -1:
  if max_blue_area >= LINE_THRESH:
  turning = True
```
Lastly, we end the turn when we find the second line and add one to the turn count.
```python
if turning:
  if track_dir == 1:
    if max_blue_area >= LINE_THRESH:
        turning = False
        turn_count += 1
  if track_dir == -1:
    if max_orange_area >= LINE_THRESH:
        turning = False
        turn_count += 1
```
After 12 turns, we start incrementing a variable called actions_to_straight. After each loop, we increase the variable by one until it reaches a threshold. We then stop the code, allowing the car to end in the middle of the starting section instead of right after detecting the final line.
```python
if max_turns <= turn_count:
  actions_to_straight += 1
  if actions_to_straight >= 25:  
    board.pwm_servo_set_position(0.1, [[1, 1500]])
    board.pwm_servo_set_position(0.1, [[2, 1500]])
    break
```

Open Challenge:
- wall following based on  wall difference
- PID (only proportional and derivative gain) on curr diff and last diff
- if ratio of wall rois filled a lot, this means walls are closer, use lower PID to prevent osccilation
- turning is based solely on wall difference
- lines are used to count turns
- once 12 turns are up. "timer" runs for actions to straight
<br>
Obstacle challenge: <br>
starting parking algorithm:
- check left and right roi. check which one is bigger and if pass wall thresh, initiate unparking sequence
- car will end up facing the same direction it is.
- unparking seqquence is hard coded by time

Obstacle 
- ignore parkinglot by treating anything magenta as black.
2 states:
  turning: when car is within blue and orange line
  straight: when car is in straight

targets defined as: targets is the x value line that center x of pillar must hit
pillars existing defined as: pillar size > pillar threshold

ignoreing pillar based on cY (mainly cY to see how close the pillar is and if the car has passed it ) and cX



target/pillar following:
once target established based on normalized y, the error is difference between target and pillar cX value. last difference is error between current error and last error. Apply PID to these values similar to wall following

if in straight and pillars exist, follow target, ignore target at certain Y value to ensure you are not over following
if in straight and pillars dont exist, use previous open challenge wall following algorithm (different/adjusted/more sensative PID)
if in turning pillar threshold is lowered 300% to ensure ending position is optimal
if turning and pillars exist, prioritize pillar targets, ignorring pillar much later than straight to ensure full turning aorund in hard cases
if turning and pillars dont exist, turn at certain angle until second coloured line is seen
