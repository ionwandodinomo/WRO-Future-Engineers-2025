
<br>

Engineering Documentation 🛠️
======

<br>

> This repository details Team Tralalero Tralala’s building and programming process in the 2025 WRO Future Engineers Competition.

---

<br>

## Content of Repository 📘
* `models` - 3D CAD files
* `others` - Other essential files
* `schemes` - Electrical schematics
* `src` - Main and other programs to run/control software
* `t-photos` - Team photos
* `v-photos` - Robot photos
* `video` - Video demonstration

<br>

## Content of Documentation 📖
* **[The Team](#the-team-boygirlboy)**
* **[Challenge Overview](#challenge-overview-)**
	* `[Explain the challenge itself. Explain basic robot specs like os, modules, etc.] {EDIT}`
* **[Mechanical Breakdown :wrench:](#mechanical-breakdown-wrench)**
	* `[MAKE SURE TO ADD POSSIBLE IMPROVEMENTS] {EDIT}`
* **[Mobility Management :car:](#mobility-management-car)**
	* [The Chassis](#the-chassis)
		* `[Introduce rear wheel drive system, independent from free-spinning front wheels.] {EDIT}`
	* [Drive System](#drive-system)
	* [Steer System](#steer-system)
* **[Power and Sense Management :zap:](#power-and-sense-management-zap)**
	* [The Battery](#the-battery)
	* [Electrical Wiring](#electrical-wiring)
	* [The Sensors](#the-sensors)
* **[Software :computer:](#software-computer)**
	* [Open Challenge](#open-challenge)
	* [Obstacle Challenge](#obstacle-challenge)
* **[Assembly Instructions](#assembly-instructions)**
* **[Youtube Videos](#youtube-videos)**
<br>

## The Team :boy::girl::boy:
`sum sum team introduction sum sum {EDIT}`
<p align="center">
	<img src="t-photos/serious1.JPEG" width="600"/> 
  <br>
  <em>Serious Photo</em>
</p>

### Team Members
 - John Weng (left)
 - Sunni Xue (middle)
 - Eric Rao (right)

|  | Funny Photos |  |
|:---:|:---:|:---:|
| <img src="t-photos/funny1.JPEG" width="350"/> | <img src="t-photos/funny2.JPEG" width="350"/> | <img src="t-photos/funny3.JPEG" width="350"/> |

<br>

### About us

| Name | About us |
| --- | --- |
| John Weng | Hey! I'm a grade 12 student from Maple High School in Ontario, Canada, and this is my second year doing the Future Engineers Category for WRO. I love messing with mechanics and designing solutions to problems. I love playing sports, especially badminton and volleyball. |
| Sunni Xue | Hi, I'm a grade 11 in the GTA! I've been coding since 6th grade, where I learned python to build my first project. I love playing sports and reading in my free time :) |
| Eric Rao | guys add stuff herepls |

## Challenge Overview 📑
`describe the challenge itself here, and include basic robot specifications like operating system, modules, sensors, etc. {EDIT}`

---

<br>

## Mechanical Breakdown :wrench:

<br>

### Materials List

| Component | Type/Category | Link / Notes |
|:---------:|:------------:|:-------------|
| Raspberry Pi 5 (8 GB) | SBC / Processor | — |
| RRC Lite Controller | Controller | [Link](https://www.hiwonder.com/products/rrc-lite?srsltid=AfmBOorP3iSszjlniFgA0gsbfA9aUH3UK0MWuyPeuTUcW6RMQjcizhmE) |
| GT24 B Differential Gear Set | Mechanical / Drive | [Link](https://carisma-shop.com/collections/gt24-spares/products/gt24-b-differential-gear-set) |
| GT24 B Spur Gear 58t | Mechanical / Drive | [Link](https://carisma-shop.com/collections/gt24-spares/products/gt24-b-spur-gear-58t) |
| GT24 SUBARU WRC WHEELS & TIRES SET | Drive | [Link](https://carisma-shop.com/collections/gt24-spares/products/gt24-subaru-wrc-wheels-tires-set-mounted) |
| GT24 WHEEL Outdrive Set | Drive| [Link](https://carisma-shop.com/collections/gt24-spares/products/gt24-b-axle-outdrive-set) |
| GT24 B Plastic Wheel lock nut set | Drive | [Link](https://carisma-shop.com/collections/gt24-spares/products/gt24-b-plastic-screw-on-wheel-nuts-x4) |
| Furitek Micro Komodo 1212 3450KV Brushless Motor | Motor | [Link](https://furitek.com/products/furitek-micro-komodo-1212-3456kv-brushless-motor-with-15t-steel-pinion-for-fury-wagon-fx118) |
| Furiteck Lizard Pro 30A/50A Brushless ESC | ESC | [Link](https://furitek.com/products/combo-of-furitek-lizard-pro-30a-50a-brushed-brushless-esc-for-axial-scx24-with-bluetooth) |
| Gens Ace 2S1P 1300mAh 7.4V battery | Battery | [Link](https://www.adrenalinehobby.com/products/gens-ace-g-tech-1300mah-2s-7-4v-25c-lipo-deans-plug?_pos=1&_sid=dde29d30b&_ss=r) |
| Sun Founder SG90 Micro Digital 9G Servo Motor | Servo | [Link](https://www.sunfounder.com/products/sg90-micro-digital-servo?srsltid=AfmBOop4G8SB4zvimDdmDlNUaAaMoN5-eXqEeMZD69HXEi-1QH7Qkzmw) |
| Mini Rocker Switch | Switch | — |
| Raspberry Pi Camera Module 8 MP | Sensor / Camera | [Link](https://www.amazon.ca/TUOPUONE-Compatible-Raspberry-MIPI-CSI-Interface/dp/B0CPTPJLXL?th=1) |
| Micro SD card | Storage | — |
| LDRobot LD19 lidar kit | Sensor / Lidar | [Link](https://www.aliexpress.com/item/1005003012681021.html?spm=a2g0o.order_list.order_list_main.11.7a3b18028WK12R) |

<br>

---

<br>

Our car uses primarily **3D printed PLA filament** as the structural material. We used the **[Prusa MK4S](https://www.prusa3d.com/product/original-prusa-mk4s-3d-printer-5/)** from Prusa Research, as well as the ENDER 3 S1. 3D printing allows precise designing of parts, and PLA is readily available and easily accessible for commercial 3D printing. After assessment, it was chosen over other materials such as ABS or PETG. These filaments are also all generally lighter, and much more customizable than other common materials, such as LEGO.

<br>

| Material | Pros | Cons |
|----------|--------|---------|
| **PLA** | - Easy to print<br>- Accurate <br>- Available<br>- Good surface finish | - Brittle<br>- Low heat resistance|
| **ABS** | - Strong<br>- Heat resistant<br>- Slightly flexible | - Warps easily<br>- Emits fumes<br>- Harder to print |
| **PETG** | - Strong<br>- Decent heat resistance<br>| - Stringing issues<br>- Slower prints<br>- Less precise |
| **Nylon** | - Very durable| - Hard to print<br>- Hygroscopic (*retains moisture*) <br>- High temp needed |
| **Carbon Fiber Filaments** | - Rigid<br>- Lightweight<br>- Strong | - Expensive<br>- Requires special setup<br>- Brittle |

<br>

In this particular environment, the cons of PLA aren't drastically impactful. The robot will not have to endure high temperatures, and load-bearing/flexibility is not a priority.

<br>

--- 

<br>

Our CAD softwares of choice were **[Onshape](https://www.onshape.com/en/)**, as well as **[TinkerCAD](https://www.tinkercad.com/)** for simpler geometry. We strongly recommend these two softwares for basic component design, as they are generally easy to learn, and provide convenient online cloud storage.

Provided below are models of the 3D printed parts, individually, as well as on the final car. Note the .stl files can all be found in the `models` folder of this repository.

<br>


### 3D Printed Parts  

<table>
  <tr>
    <td align="center" width="25%">
      <b>Front</b><br>
      <img src="other/placeholder.png" width="200"/>
    </td>
    <td align="center" width="25%">
      <b>Back</b><br>
      <img src="other/placeholder.png" width="200"/>
    </td>
    <td align="center" width="25%">
      <b>Main Base Plate</b><br>
      <img src="other/placeholder.png" width="200"/>
    </td>
    <td align="center" width="25%">
      <b>Connective Base Plate</b><br>
      <img src="other/placeholder.png" width="200"/>
    </td>
  </tr>
  <tr>
    <td align="center" width="25%">
      <b>Camera Mount</b><br>
      <img src="other/placeholder.png" width="200"/>
    </td>
    <td align="center" width="25%">
      <b>ESC Holder</b><br>
      <img src="other/placeholder.png" width="200"/>
    </td>
    <td align="center" width="25%">
      <b>Battery Tray</b><br>
      <img src="other/placeholder.png" width="200"/>
    </td>
    <td align="center" width="25%">
      <b>Motor Bracket</b><br>
      <img src="other/placeholder.png" width="200"/>
    </td>
  </tr>
  <tr>
    <td align="center" width="25%">
      <b>Servo Mount</b><br>
      <img src="other/placeholder.png" width="200"/>
    </td>
    <td align="center" width="25%">
      <b>Front Bumper</b><br>
      <img src="other/placeholder.png" width="200"/>
    </td>
    <td align="center" width="25%">
      <b>Rear Bumper</b><br>
      <img src="other/placeholder.png" width="200"/>
    </td>
    <td align="center" width="25%">
      <b>Side Guard (Left)</b><br>
      <img src="other/placeholder.png" width="200"/>
    </td>
  </tr>
  <tr>
    <td align="center" width="25%">
      <b>Side Guard (Right)</b><br>
      <img src="other/placeholder.png" width="200"/>
    </td>
    <td align="center" width="25%">
      <b>Sensor Bracket</b><br>
      <img src="other/placeholder.png" width="200"/>
    </td>
    <td align="center" width="25%">
      <b>Wheel Hub</b><br>
      <img src="other/placeholder.png" width="200"/>
    </td>
    <td align="center" width="25%">
      <b>Spacer / Riser</b><br>
      <img src="other/placeholder.png" width="200"/>
    </td>
  </tr>
</table>  

---

<br>

### Full Assembly Views  

#### Printed Assembly  

<table>
  <tr>
    <td align="center" width="33%">
      <b>Front View</b><br>
      <img src="other/placeholder.png" width="200"/>
    </td>
    <td align="center" width="33%">
      <b>Back View</b><br>
      <img src="other/placeholder.png" width="200"/>
    </td>
    <td align="center" width="33%">
      <b>Side View</b><br>
      <img src="other/placeholder.png" width="200"/>
    </td>
  </tr>
  <tr>
    <td align="center" width="33%">
      <b>Top View</b><br>
      <img src="other/placeholder.png" width="200"/>
    </td>
    <td align="center" width="33%">
      <b>Isometric View</b><br>
      <img src="other/placeholder.png" width="200"/>
    </td>
    <td align="center" width="33%">
      <b>Exploded View</b><br>
      <img src="other/placeholder.png" width="200"/>
    </td>
  </tr>
</table>  

#### With Components  

<table>
  <tr>
    <td align="center" width="33%">
      <b>Front View</b><br>
      <img src="other/placeholder.png" width="200"/>
    </td>
    <td align="center" width="33%">
      <b>Back View</b><br>
      <img src="other/placeholder.png" width="200"/>
    </td>
    <td align="center" width="33%">
      <b>Side View</b><br>
      <img src="other/placeholder.png" width="200"/>
    </td>
  </tr>
  <tr>
    <td align="center" width="33%">
      <b>Top View</b><br>
      <img src="other/placeholder.png" width="200"/>
    </td>
    <td align="center" width="33%">
      <b>Isometric View</b><br>
      <img src="other/placeholder.png" width="200"/>
    </td>
    <td align="center" width="33%">
      <b>Exploded View</b><br>
      <img src="other/placeholder.png" width="200"/>
    </td>
  </tr>
</table>  
=======

### Full Assembly Views

#### Full Assembly (Printed Parts Only)

| Top | Bottom | Front |
|:---:|:---:|:---:|
| <img src="other/placeholder.png" width="250"/> | <img src="other/placeholder.png" width="250"/> | <img src="other/placeholder.png" width="250"/> |

| Back | Left | Right |
|:---:|:---:|:---:|
| <img src="other/placeholder.png" width="250"/> | <img src="other/placeholder.png" width="250"/> | <img src="other/placeholder.png" width="250"/> |

<br>

---

<br>

#### Full Assembly (with Components)

| Top | Bottom | Front |
|:---:|:---:|:---:|
| <img src="v-photos/CAR_TOP.jpg" width="250"/> | <img src="v-photos/CAR_BOTTOM.jpg" width="250"/> | <img src="v-photos/CAR_FRONT.jpg" width="250"/> |

| Back | Left | Right |
|:---:|:---:|:---:|
| <img src="v-photos/CAR_BACK.jpg" width="250"/> | <img src="v-photos/CAR_LEFT.jpg" width="250"/> | <img src="v-photos/CAR_RIGHT.jpg" width="250"/> |


These parts were designed with **modularity** in mind, meaning each section of the car can function more or less individually. They piece together smoothly as separate parts, instead of a single printed chassis, as originally planned. Screws were avoided as much as possible, as they are less reflective of this idea, but regardless used when necessary. 

<p align="center">
  <img src="other/placeholder.png" width="300" alt="Previous Version"/>
  <br>
  <em>Original concept of a single chassis plate</em>
</p>


We found this approach much more convenient while testing, as it allows for changes to be made to a specific part of the car, and easily replaced and updated without collateral impact to the rest of the body. In other words, an **iterative** approach.

<br>

### Possible Improvements
- Upgrading print system to include **PETG** or **carbon fiber** filaments for more durable, high-quality parts.
- Changing connection points to **snap-fits**, **press-fits**, etc, rather than screws, to simplify maintenance and quick iteration.
- Optimized print orientation to reduce warping and the need for potentially inaccurate supports.

<br>

---

<br>

## Mobility Management :car:

For the use of this competition, movement precision and responsiveness are the top priorities.

---

<br>

### The Chassis
The chassis is located in the center of the car. It provides a strucutre to lay the majority of electrical components onto, namely the **two boards** as well as the **LIDAR**. It also serves as a medium to connect the front and rear wheel systems. The chassis itself is split into two seperate plates, that align together to form the abdomen of the car.

<br>

#### Chassis Models
<table>
  <tr>
    <td align="center" width="33%">
      <b>Main Plate</b><br>
      <img src="other/placeholder.png" width="250"/>
    </td>
    <td align="center" width="33%">
      <b>Connective Plate</b><br>
      <img src="other/placeholder.png" width="250"/>
    </td>
    <td align="center" width="33%">
      <b>Together</b><br>
      <img src="other/placeholder.png" width="250"/>
    </td>
  </tr>
</table>

<br>

Again, the choice of 3D printed PLA filament allows this precise jointery of components. They fit snugly together, and the friction also helps keep tight connection points along the entirety of the chassis. 

After some trials with the 3D printer, our final print settings include a **0.4mm nozzle**, **~50% infill**, **brim disabled**, and **snug** supports. These specific choices enable a strudy, slightly flexible base, and remove the "junk" filament that might otherwise be printed with brim enabled or other kinds of supports.

Another consideration improving the **balance** of the robot, which directly influences the consistency of our movement. Two vital aspects are maintaining a low, as well as centered point of mass. In another words, keeping our components, particulary the heavy ones, as low and as centralized as possible. For example, the boards, which are located at the very base and center of the car.

<br>

<p align="center">
  <img src="other/placeholder.png" width="300" alt="Previous Version"/>
  <br>
  <em>Location of the Raspberry Pi and Arduino</em>
</p>

<br>

#### Camera Mount

<table>
  <tr>
    <td width="50%">
      One restriction to take into account is the minimum height instated by the <b>camera</b>.  
      <br><br>
      We found that it had to be placed fairly high up, with a generous angle downwards, in order to consistently capture all the challenge objects.  
      <br><br>
      A <b>modular</b> design was implemented to test with different heights and angles, but the consequence of a high center of mass was left overlooked.  
    </td>
    <td width="50%" align="center">
      <img src="other/placeholder.png" width="300"/>
    </td>
  </tr>
</table>  

<br>

#### Possible Improvements
- Optimizing the location of our components, such as lowering the **LIDAR** or centralizing the **battery** for an ideal center of mass.
- Adding **suspension** or other shock-absorbant materials for steadier and reliable conditions for the sensors
- Design a housing to protect sensitive components from dust or other external debris.
- Lowering the height of the camera; perhaps by using a more **LIDAR** dominant approach.

<br>

---

<br>

### Drive System

The robot features a **rear-wheel drive (RWD)** layout powered by the **Furitek Micro Komodo Motor**, paired with the **Furitek Lizard Pro ESC**. A **12:58 gear ratio** was selected to optimize torque delivery for obstacle navigation.  

<br>

#### Why Rear-Wheel Drive?  
- **Lightweight Drive System**: The front wheels remain free for a simpler, lighter system.  
- **Efficient Transfer**: Torque from the motor is delivered directly to the gearbox, minimizing drivetrain energy losses.  
- **Modularity**: Allows swapping motor mounts and experimenting with different gear ratios without redesigning the entire chassis.  

<br>

#### Motor Specifications

<table>
  <tr>
    <td align="center" width="50%">
      <b>Furitek Micro Komodo Motor</b><br>
      <img src="other/motor.png" width="250"/>
    </td>
    <td align="left" width="50%">
      <ul>
        <li><b>Type:</b> Brushless DC Motor</li>
        <li><b>Voltage:</b> 7.4V nominal</li>
        <li><b>Max Current Draw:</b> ~5A</li>
        <li><b>ESC:</b> Furitek Lizard Pro</li>
        <li><b>Gear Ratio:</b> 12:58</li>
        <li><b>Drive Layout:</b> Rear-Wheel Drive (RWD)</li>
      </ul>
    </td>
  </tr>
</table>  

<br>

---

<br>

#### Mounting and Layout  

- Motor secured to the **rear chassis plate**

<br>

### Steer System

<br>

## Power and Sense Management :zap:
### The Battery
The [Gens Ace 2S1P 1300mAh 7.4V battery](https://www.adrenalinehobby.com/products/gens-ace-g-tech-1300mah-2s-7-4v-25c-lipo-deans-plug?_pos=1&_sid=dde29d30b&_ss=r) is overkill for the power supply of our car. This battery has a continuous discharge rate of 45C, delivering more than enough power for the peak demand of our electronics, including the SG90 Servo, Furitek motor, ESC, Raspberry Pi 5 and the RRC Lite Controller. The ESC, connected directly to the controller, regulates the voltage to the motor, avoiding brownouts even under full load.



#### Power Ratings

| Component | Voltage | Max Current Draw |
|:---------:|:-------:|:---------------:|
| RRC Lite Controller | 5 V | 0.5 A |
| YDLidar T-mini | 5 V | 0.3 A |
| Furitek Micro Komodo Motor | 7.4 V | 5 A |
| Furitek Lizard Pro ESC | 8.4 V | 50 A |
| SG90 Servo Motor | 5 V | 0.25 A |
| Pi Camera | 5 V | 0.25 A |
| MicroSD, LEDs, and Speakers | 5 V | 0.2 A |

---

#### Total Power Draw

| Category | Value |
|:--------:|:-----:|
| Average Current Draw | 10.5 A |
| Battery Output Capacity | 32.5 A |
| Headroom | 22 A |


### Electrical Wiring


### The Sensors
The car gets input from the [Raspberry 5 Camera Module 8 MP MIPI-CSI Interface](https://www.amazon.ca/TUOPUONE-Compatible-Raspberry-MIPI-CSI-Interface/dp/B0CPTPJLXL?th=1), the inbuilt gyro sensor in the raspberry pi, and the [LDRobot D500 lidar kit](https://www.amazon.ca/LDROBOT-Outdoor-Navigation-Scanning-Support/dp/B0DDKXQ23R). We can use these 3 in conjunction, with the camera detecting colour, walls, and the lidar detecting walls and parking spaces. The gyro sensor aids the other two sensors in the precise movements, allowing for greater control over the car

<br>

## Software :computer:

### Image Preprocessing
In both challenges, we used a Raspberry Pi camera with cv2.contours to locate the various obstacles and walls. The camera has specific regions of interest (ROIs) for each "object type", allowing us to cut out noise and unnecessary uncertain variables. Additionally, only drawing contours in specific ROIs allows us to combine ROIs to create more "fitting" shapes (eg. 2 for each wall due to perspective). 

The open challenge utilized 3 colours: black (wall), orange (line), blue (line).
The obstacle challenge utilized 6 colours: black (wall), orange (line), blue (line), magenta (parking lot), red (pillar), green (pillar).

All issues experienced did not significantly affect open challenge due to its simple nature.

Originally, the contours were drawn with HSV (hue, saturation, value) ranges. Each colour had one ranges: an upper and lower boundary. However, due to the nature of hsv, red required 2 ranges and changes in lighting proved significant for the accuracy of the contours. Additionally, distinguishing orange, red, and magenta proved challenging in certain lightings. [insert hsv range gui]

Since HSV ranges proved difficult, we made the switch over to LAB (lightness, red-green axis, blue-yellow axis), which is more tolerant of lighting changes. This also allowed for the red contour to only use two ranges.

In addition to the switch to LAB, several more filters were applied and tested in an effort to increase colour detection accuracy


Using these ranges, a colour mask could be created and applied onto each frame. Using a mask also allowed us to combine colours in contours. For example, using the bitwise or operator, we could combine black and magenta contours to be treated as a wall, when not in parking sequence.
To increase the consistency, we switched to LAB ranges as they were more accurate when checking the hues. Most regions of interest only check for the largest contour in their region, as smaller contours are either insignificant at the point or not a part of the track.


Saturation: X
- We had the idea to increase the saturation of the frame by units of magnitude to increase visibilitly between red and orange, especially in dull lighting
- However, this messed up other colours, more specificall white, and it would show up "rainbow" due to the undertones being more visible
- insert image of gui sunni do latoor ref/src collour

filter/colour normalization: !
- Due to concerns with lighting, we created a mask based on the comparison between ideal colour and frame capture colour of the white map.
- This mask could then be applied across the entirety of the frame.
- This means if there was yellow lighting, the mask would cancel it out, etc. This also allowed auto brightness adjustment
- Additionally, this meant LAB colour ranges were more stable, with less changes needed day-of
- insert image of gui sunni do latoor ref/src collour

colour flattening: X
- make similar colours flatten, shadow less
- too simplistic and cpu intensive
- insert image of gui sunni do latoor ref/src collour
blur: !
- gausian blur bruvski cuh less sharp ig
- insert image of gui sunni do latoor ref/src collour

Unexpectedly, frame rate also played a large role in contour detection:
- It was observed/calculated that the process bottleneck at ~64 fps
- Frame rates 25, 30, 50, 75 were tested
	- 25: too slow
 	- 30: perfect
  	- 50: messed with exposure, creating a much darker capture than reality
  	- 75: too fast
  
Camera settings: i aint explaining thiss brother
```py
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
```

### Open Challenge
#### Overview of Challenge
The open challenge is a time attack round, where each team has just 3 minutes to drive 3 laps autonomously on a track. Every round of the challenge, the layout is randomized, where each straight section can either consist of wide walls, leaving a large gap between the inside or outside walls, or narrow walls, giving the car much less space to manoeuvre. The track direction is also randomized. The goal of the open challenge is to finish the laps as fast and consistently as possible, not touching the inside walls or crashing the vehicle.

The difficulty of the challenge comes from the changes in the width of the track, where the track turns from narrow to wide or from wide to narrow. This drastic change in the track can confuse the car, creating instances where it is difficult to ascertain the correct direction of the track or for the car to recognise the walls.
<p align="center">
  <img src="/other/Map with Sections.jpg" height="350">
  <br>
  <em>Overhead view of map with sections</em>
</p>

#### Our Solution
To break down the challenge into manageable parts, we thought of it as three parts. These three parts are comprised of turning, driving in the straights, and counting the turns. In all three parts, 6 regions of interest were used. 4 of these regions are tracking the walls, using a LAB or HSV range to check for the black colour of the walls, returning a number for the total number of pixels of the left two regions, and a number for the total number of pixels of the right two regions. The last two regions of interest are in the bottom left and right, used to track the blue and orange lines on the track, allowing us to ascertain the correct direction and the corners of the track. Only one of the two regions are used depending on the direction of the track.
<p align="center">
<img src="/other/open straight.PNG" height="250">
  <br>
  <em>Open challenge ROIS</em>
</p>

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
      <img src="/other/open straight.PNG" width="350"/>
    </td>
    <td align="center" width="33%">
      <b>Starting Turn</b><br>
      <img src="/other/open start turn.PNG" width="350"/>
    </td>
    <td align="center" width="33%">
      <b>Ending Turn</b><br>
      <img src="/other/open end turn.PNG" width="350"/>
    </td>
  </tr>
</table>

<br>

### Obstacle Challenge
#### Overview of Challenge

The obstacle challenge is a much more difficult version of the open challenge, emphasised by being worth over double the points as the open challenge. In this challenge, the walls are not randomized, but instead introduce red and green pillars that instruct the car on lane positioning. The car must pass red pillars on the right side and green pillars on the left side. After 3 laps, the car must parallel park in the given parking space. The starting section, driving section and position of traffic signs are randomized. The parking lot is always in the starting section, with an additional option to start in the parking lot for extra points.

The addition of these extra features comes with its own set of difficulties. The car must reliably detect the traffic signs and make split-second decisions to avoid breaking the rules. The car must drive smoothly and precisely to execute proper turns and complete parallel parking. It also must be able to make decisions on its own, as the positions and direction of the track change every round.
<p align="center">
<img src="/other/wromap obstacle.jfif" height="250">
  <br>
  <em>ADD obstacle challenge image</em>
</p>

#### Our Solution
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
      <img src="/other/obstacle following pillar.PNG" width="350"/>
    </td>
    <td align="center" width="33%">
      <b>Ignoring Close Pillar</b><br>
      <img src="/other/obstacle ignoring close.PNG" width="350"/>
    </td>
    <td align="center" width="33%">
      <b>Ignoring Far Pillar</b><br>
      <img src="/other/obstacle ignoring far.PNG" width="350"/>
    </td>
  </tr>
</table>

<br>

## Assembly Instructions 🛠️


## Youtube Videos
[Open Challenge Video](https://youtu.be/-PhhBH6H1vY) 

[Obstacle Challenge Video](https://youtu.be/-8iTvhqq1Cs) 
