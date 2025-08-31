
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
	* [Initialization](#initialization)
	* [Object Detection](#object-detection)
	* [Wall Management and Turning](#wall-management-and-turning)
* **[Assembly Instructions](#assembly-instructions)**

<br>

## The Team :boy::girl::boy:
`sum sum team introduction sum sum {EDIT}`

---

<br>

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
| Furitek Micro Komodo 1212 3450KV Brushless Motor | Motor | [Link](https://furitek.com/products/furitek-micro-komodo-1212-3456kv-brushless-motor-with-15t-steel-pinion-for-fury-wagon-fx118) |
| Furiteck Lizard Pro 30A/50A Brushless ESC | ESC | [Link](https://furitek.com/products/combo-of-furitek-lizard-pro-30a-50a-brushed-brushless-esc-for-axial-scx24-with-bluetooth) |
| Gens Ace 2S1P 1300mAh 7.4V battery | Battery | [Link](https://www.adrenalinehobby.com/products/gens-ace-g-tech-1300mah-2s-7-4v-25c-lipo-deans-plug?_pos=1&_sid=dde29d30b&_ss=r) |
| Sun Founder SG90 Micro Digital 9G Servo Motor | Servo | [Link](https://www.sunfounder.com/products/sg90-micro-digital-servo?srsltid=AfmBOop4G8SB4zvimDdmDlNUaAaMoN5-eXqEeMZD69HXEi-1QH7Qkzmw) |
| Mini Rocker Switch | Switch | — |
| Raspberry Pi Camera Module 8 MP | Sensor / Camera | [Link](https://www.amazon.ca/TUOPUONE-Compatible-Raspberry-MIPI-CSI-Interface/dp/B0CPTPJLXL?th=1) |
| Micro SD card | Storage | — |
| LDRobot D500 lidar kit | Sensor / Lidar | [Link](https://www.amazon.ca/LDROBOT-Outdoor-Navigation-Scanning-Support/dp/B0DDKXQ23R) |

<br>

---

<br>

Our car uses primarily **3D printed PLA filament** as the structural material. We used the **[Prusa MK4S](https://www.prusa3d.com/product/original-prusa-mk4s-3d-printer-5/)** from Prusa Research, as well as the `[JOHN'S PRINTER].` 3D printing allows precise designing of parts, and PLA is readily available and easily accessible for commercial 3D printing. After assessment, it was chosen over other materials such as ABS or PETG. These filaments are also all generally lighter, and much more customizable than other common materials, such as LEGO.

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

### Drive System

The robot uses a **rear-wheel drive (RWD) configuration** powered by the **Furitek Micro Komodo Motor**.  
This choice emphasized simplicity, direct torque delivery, and modularity when testing drivetrain iterations.

<br>

#### Why Rear-Wheel Drive?  
- **Lightweight Drive System** – The front wheels remain free for a simpler, lighter system.
- **Efficient Transfer** – Torque from the motor is delivered directly to the gearbox, minimizing drivetrain energy losses.
- **Modularity** – Allows swapping motor mounts and experimenting with different gear ratios without redesigning the entire chassis.  

<br>

#### The Furitek Micro Komodo
- High torque output for low-speed movement.
- Compact size compatible with the chassis.  
- Paired effectively with the **Furitek Lizard Pro ESC**

<br>

<table>
  <tr>
    <!-- Motor Image -->
    <td align="center" width="50%">
      <img src="other/motor.png" width="250"/><br>
      <b>Furitek Micro Komodo Motor</b>
    </td>
    
    <!-- Motor Specs -->
    <td align="left" width="50%">
      <b>Specifications</b>
      <ul>
        <li>Voltage: 7.4V</li>
        <li>Max Current Draw: 5A</li>
        <li>Configuration: Rear-Wheel Drive</li>
        <li>Gear Ratio: 12:58</li>
        <li>Compact size for small chassis integration</li>
        <li>Paired with Furitek Lizard Pro ESC</li>
      </ul>
    </td>
  </tr>
</table>

<br>

#### Challenges & Tradeoffs  
- **Rear Weight Bias**: Locating the motor at the back concentrated mass near the rear axle, reducing overall balance when paired with the elevated camera mount.  
- **Torque vs. Wheel Slip**: The 12:58 gear ratio gave strong low-end torque, but under sharp turns the excess torque occasionally caused the rear wheels to slip.  
- **Space Limitation**: The rear drive motor reduced available space for additional rear-mounted components (e.g. sensors or battery).  

---

#### Possible Improvements  
- Experiment with **alternative gear ratios** (e.g. taller gearing for less wheel slip).  
- Add a **gear reduction stage** with better torque distribution.  
- Test **dual-motor drive** (left and right independently powered) to allow differential steering as a backup to front-servo steering.  
- Lower the motor position relative to the axle to reduce **center of mass height**.  
- Consider a **front-mounted battery** to counterbalance the rear-biased weight distribution.  

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
### Initialization

### Object Detection

### Wall Management and Turning

<br>

## Assembly Instructions 🛠️

