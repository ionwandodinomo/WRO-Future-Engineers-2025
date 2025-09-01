Control software
====

This directory must contain code for control software which is used by the vehicle to participate in the competition and which was developed by the participants.

All artifacts required to resolve dependencies and build the project must be included in this directory as well.

In both challenges, we used a Raspberry Pi camera with cv2.contours to find the different obstacles and walls. The camera has specific regions of interest, allowing each to check for a separate colour, as well as being able to add as many regions of interest as we want. These regions originally checked colours with HSV ranges. Each colour has two, one max and one minimum, except for red, with 4 ranges due to the hue showing up twice in HSV. Any pixel detected that was between any of the ranges was counted as that colour and created a mask for the contour. To increase the consistency, we switched to LAB ranges as they were more accurate when checking the hues. Most regions of interest only check for the largest contour in their region, as smaller contours are either insignificant at the point or not a part of the track.

## Open Challenge
### Overview of Challenge
The open challenge is a time attack round, where each team has just 3 minutes to drive 3 laps autonomously on a track. Every round of the challenge, the layout is randomized, where each straight section can either consist of wide walls, leaving a large gap between the inside or outside walls, or narrow walls, giving the car much less space to manoeuvre. The track direction is also randomized. The goal of the open challenge is to finish the laps as fast and consistently as possible, not touching the inside walls or crashing the vehicle.

The difficulty of the challenge comes from the changes in the width of the track, where the track turns from narrow to wide or from wide to narrow. This drastic change in the track can confuse the car, creating instances where it is difficult to ascertain the correct direction of the track or for the car to recognise the walls.
<p align="center">
  <img src="/other/Map with Sections.jpg" height="350">
  <br>
  <em>Overhead view of map with sections</em>
</p>

### Our Solution
To break down the challenge into manageable parts, we thought of it as three parts. These three parts are comprised of turning, driving in the straights, and counting the turns. In all three parts, 6 regions of interest were used. 4 of these regions are tracking the walls, using a LAB or HSV range to check for the black colour of the walls, returning a number for the total number of pixels of the left two regions, and a number for the total number of pixels of the right two regions. The last two regions of interest are in the bottom left and right, used to track the blue and orange lines on the track, allowing us to ascertain the correct direction and the corners of the track. Only one of the two regions are used depending on the direction of the track.
<p align="center">
<img src="/other/open straight.PNG" height="250">
  <br>
  <em>Open challenge ROIS</em>
</p>

#### Turning
The turning of the car is based on the threshold of the inside wall. Based on the direction of the track, when the number of pixels of a wall reaches below a certain threshold, we start the turn by locking the steering angle until the number of pixels in the inside wall climbs back above the threshold. This allows us to adjust during turns, signalling to start a turn when we detect that a wall disappears, usually indicating when the inside wall ends.
```python
if left_area < WALL_THRESH:
  angle = TURN_ANGLE

elif right_area < WALL_THRESH:
  angle = -TURN ANGLE
```
Past this initial turn, the rest of the turn relies on the angle calculating method in the straights.
#### Straights
Driving in the straights uses a PD calculating algorithm to find the angle the car should turn. This PD algorithm is comprised of proportional and derivative angle calculating, comprised of two constants: PD and PG. First, we find the difference in the number of pixels of the wall and multiply it by the PD, and add the variation of the last two differences multiplied by the PD. The final result is used as the angle and allows the car to adjust to the walls of the track, making sure we don't veer into a crash, while adjusting based on the past patterns.
```python
curr_diff = right_area-left_area
angle = int((curr_diff * PG + (curr_diff-last_diff) * PD))
```
However, to make the car turn earlier, we had to increase the two constants to make the car more sensitive to changes. This caused oscillations in the straights due to the high sensitivity. To fix this issue, when we detect a large amount of pixels in both walls, we assume we are in a straight section and use a lower PD and PG.
```python
curr_diff = right_area-left_area
if right_area > 1250 and left_area > 1250:
  angle = int((curr_diff * PGLOW + (curr_diff-last_diff) * PDLOW))

else:
  angle = int((curr_diff * PG + (curr_diff-last_diff) * PD))

last_diff = curr_diff
```
This lower PD substantially reduces oscillation in the straights, making the start of turns more reliable and consistent.
#### Turn Counting
Turn counting is made simple by the lines on the track. When we detect the very first line, the colour of this line indicates the direction of the track. Starting from then, every time we see that first colour followed by the second colour, we add one to the turn count.
First, we find the direction of the track from the first line.
```python
if track_dir == 0:

  if max_orange_area >= LINE_THRESH:
  track_dir = 1

elif max_blue_area >= LINE_THRESH:
  track_dir = -1
```
For all subsequent turns, we begin the turn by detecting the line we first saw on the track.
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
<table>
  <tr>
    <td align="center" width="33%">
      <b>In a Straight Section</b><br>
      <img src="/other/open straight.PNG" width="250"/>
    </td>
    <td align="center" width="33%">
      <b>Starting Turn</b><br>
      <img src="/other/open start turn.PNG" width="250"/>
    </td>
    <td align="center" width="33%">
      <b>Ending Turn</b><br>
      <img src="/other/open end turn.PNG" width="250"/>
    </td>
  </tr>
</table>

<br>

## Obstacle Challenge
### Overview of Challenge

The obstacle challenge is a much more difficult version of the open challenge, emphasised by being worth over double the points as the open challenge. In this challenge, the walls are not randomized, but instead introduce red and green pillars that instruct the car on lane positioning. The car must pass red pillars on the right side and green pillars on the left side. After 3 laps, the car must parallel park in the given parking space. The starting section, driving section and position of traffic signs are randomized. The parking lot is always in the starting section, with an additional option to start in the parking lot for extra points.

The addition of these extra features comes with its own set of difficulties. The car must reliably detect the traffic signs and make split-second decisions to avoid breaking the rules. The car must drive smoothly and precisely to execute proper turns and complete parallel parking. It also must be able to make decisions on its own, as the positions and direction of the track change every round.
<p align="center">
<img src="/other/wromap obstacle.jfif" height="250">
  <br>
  <em>ADD obstacle challenge image</em>
</p>

### Our Solution
We used the same 4 wall regions of interest and 2 line detecting regions. To check for pillars, we included one large ROI checking for both red and green pillars, covering all of the bottom and middle of the camera.
<p align="center">
<img src="/other/obstacle rois.PNG" height="250">
  <br>
  <em>ROIs in obstacle challenge</em>
</p>
We started by first attempting to follow the traffic signs in the correct direction in a straight section. To pass the pillar on the correct side, we set a target x coordinate on the camera for where we want the pillar to be when we pass it. For example, the red pillar's target is on the left side of the camera, ensuring that we pass the pillar on the right side when the pillar is at the target. For this, we use logic similar to the wall following PD algorithm in the open challenge, with a pillar PD and pillar PG. Instead of using the difference in the pixels of the wall to find the angle of the car turn, we find the error between the target and the centre x-coordinate of the pillar and multiply it by the pillar PG, then add the variation of the current and previous difference multiplied by the pillar PD. The pillar code takes priority over the wall following code from the open challenge. If there are no pillars in sight, we revert to the wall following algorithm.


```python
if selected_contour is not None:
  error = centre_pillar_x - target
  angle = -(int((error * PILLAR_PG) + ((error - last_pillar_diff) * PILLAR_PD)))
  last_diff = curr_diff
  last_pillar_diff= error

else:
  curr_diff = right_area-left_area
  angle = int((curr_diff * PG + (curr_diff-last_diff) * PD))
```
To smooth out the pillar targeting, we use dynamic gain instead of the pillar PD. This is calculated using the pillar PD, but it changes based on the distance to the pillar. If the pillar is further, the dynamic gain increases, and if the pillar is closer, the dynamic gain decreases.
```python
dynamic_gain = PILLAR_PG + (1 - normalized_y) * 0.0005
```
Even after tuning the pillar PG and PD, we still ran into an issue where our car would prioritise a pillar for too long, keeping the pillar in the target while running circles around it. To fix this issue, we created a variable called normalized_y, a ratio of how close the pillar is to the car. When the pillar approaches the car, the normalized_y increases, with a range between 0 and 1. This allowed us to set different targets based on the distance between the car and the pillar.
```python
if normalized_y < 0.2:
  RED_TARGET = 320
  GREEN_TARGET = 320

elif normalized_y < 0.4:
  RED_TARGET = 135
  GREEN_TARGET = 405

else:
  RED_TARGET = 75
  GREEN_TARGET = 565
# This is a simplified version of the targets.
```
However, due to a high PD and PG, the car would still follow the pillar in circles. To combat this problem, we ignore pillars in certain situations where they are close to the bottom camera and in the bottom left or right corners.
```python
if (centre_pillar_x  < 40 or centre_pillar_x > 600 ) and centre_pillar_y > 340:
  angle = 0
  print("ignoring pillar")
```
We also ignore the pillar following a lenient version of the condition above if the car had just finished a turn.
```python
if (centre_pillar_x  < 100 or centre_pillar_x > 540 ) and centre_pillar_y > 310 and (red_area2 < PILLAR_THRESH and green_area2 < PILLAR_THRESH) and not turning and time_after_turn >= 5:
  angle = 0
  print("ignoring pillar")
time_after_turn += 1
```
The original pillar algorithm works fine for most cases, unless the car sees a pillar in a different section above a wall. Since pillars take priority, the car attempts to get the pillar into the target, often running into a wall. We set a pillar threshold to fix this complication, we set a threshold for the pillars. Only pillars above a certain number of pixels will take priority, allowing us to only check pillars that are in the current section.
```python
if PILLAR_THRESH < cv2.contourArea(max_red_contour) > cv2.contourArea(max_green_contour):
  selected_contour = max_red_contour
  target = RED_TARGET

elif PILLAR_THRESH < cv2.contourArea(max_green_contour):
  selected_contour = max_green_contour
  target = GREEN_TARGET

else:
  selected_contour = None
```
Unfortunately, this didn't solve all our issues. Due to the random nature of the pillars, sometimes the pillars would be placed in a difficult case, where right after the turn, the pillar is set at a point that severely limits the amount of space the car has to turn. To fix these cases, we added a set angle for the car to turn at when it is within the orange and blue lines and doesn't see a pillar. This allows the car to turn earlier, seeing pillars and increasing the amount of time the car has to adjust its position.
```python
if turning:

  if track_dir==1:
    angle = -MAX_TURN_LESS

  else:
    angle = MAX_TURN_LESS
```
We also lower the pillar threshold during turns, allowing the car to straighten itself and predict the next pillars after a turn.
```python
if turning:

        PILLAR_THRESH = 150
        if track_dir == 1 and max_blue_area >= LINE_THRESH:
            turn_count += 1
            turning = False
            PILLAR_THRESH = 1200
            time_after_turn = 0

        elif track_dir == -1 and max_orange_area >= LINE_THRESH:
            turn_count += 1
            turning = False
            PILLAR_THRESH = 1200
            time_after_turn = 0
```
The turn-counting and wall following code is reused from the open challenge. To avoid the magenta parking lot, when we are not in a parking sequence, we count all magenta blocks as part of the wall, relying on the wall following algorithm to ignore the parking lot.

In short, we can use this pseudo-code to outline the overall process:
```python
if in straight:

  if pillar exists:
    follow target

    if pillar is close:
      ignore pillar
  if pillar doesn't exist:
    use wall following code
  
if turning:
  lower pillar threshold

  if pillar exists:
    prioritize pillar target
    ignore pillar later

  if pillar doesn't exist
    turn until line and end turn
```

<table>
  <tr>
    <td align="center" width="33%">
      <b>Following Pillar</b><br>
      <img src="/other/obstacle following pillar.PNG" width="250"/>
    </td>
    <td align="center" width="33%">
      <b>Ignoring Close Pillar</b><br>
      <img src="/other/obstacle ignoring close.PNG" width="250"/>
    </td>
    <td align="center" width="33%">
      <b>Ignoring Far Pillar</b><br>
      <img src="/other/obstacle ignoring far.PNG" width="250"/>
    </td>
  </tr>
</table>

<br>
