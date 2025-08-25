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
