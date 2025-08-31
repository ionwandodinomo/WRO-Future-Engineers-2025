

&nbsp;
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
* **[Challenge Overview](#challenge-overview)**
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

Our car uses primarily **3D printed PLA filament** as the structural material. We used the **[Prusa MK4S](https://www.prusa3d.com/product/original-prusa-mk4s-3d-printer-5/)** from Prusa Research, as well as the `[JOHN'S PRINTER].` 3D printing allows precise designing of parts, and PLA is readily available and easily accessible for commercial 3D printing. After assessment, it was chosen over other materials such as ABS or PETG.

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

Provided below are models of the 3D printed parts, individually, as well as on the final car.

<br>

### 3D Printed Parts

`[TAKE PICTURES OF THESE IN ONSHAPE PROBABLY]`

| Front | Back | Main Base Plate | Connective Base Plate |
|:---:|:---:|:---:|:---:|
| <img src="other/placeholder.png" width="200"/> | <img src="other/placeholder.png" width="200"/> | <img src="other/placeholder.png" width="200"/> | <img src="other/placeholder.png" width="200"/> |

| Part 5 | Part 6 | Part 7 | Part 8 |
|:---:|:---:|:---:|:---:|
| <img src="other/placeholder.png" width="200"/> | <img src="other/placeholder.png" width="200"/> | <img src="other/placeholder.png" width="200"/> | <img src="other/placeholder.png" width="200"/> |

| Part 9 | Part 10 | Part 11 | Part 12 |
|:---:|:---:|:---:|:---:|
| <img src="other/placeholder.png" width="200"/> | <img src="other/placeholder.png" width="200"/> | <img src="other/placeholder.png" width="200"/> | <img src="other/placeholder.png" width="200"/> |

<br>

---

<br>


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
| <img src="other/placeholder.png" width="250"/> | <img src="other/placeholder.png" width="250"/> | <img src="other/placeholder.png" width="250"/> |

| Back | Left | Right |
|:---:|:---:|:---:|
| <img src="other/placeholder.png" width="250"/> | <img src="other/placeholder.png" width="250"/> | <img src="other/placeholder.png" width="250"/> |

These parts were designed with **modularity** in mind, meaning each section of the car can function more or less individually. They piece together smoothly as separate parts, instead of a single printed chassis, as originally planned. Screws were avoided as much as possible, as they are less reflective of this idea, but regardless used when necessary. 

<p align="center">
  <img src="other/placeholder.png" width="300" alt="Previous Version"/>
  <br>
  <em>Original concept of a single chassis plate</em>
</p>


We found this approach much more convenient while testing, as it allows for changes to be made to a specific part of the car, and easily replaced and updated without collateral impact to the rest of the body. In other words, an **iterative** approach.

<br>

---

<br>

### Possible Improvements
- Upgrading print system to include **PETG** or **carbon fiber** filaments for more durable, high-quality parts.
- Changing connection points to **snap-fits**, **press-fits**, etc, rather than screws, to simplify maintenance and quick iteration.
- Optimized print orientation to reduce warping and the need for potentially inaccurate supports.

<br>

---

<br>

## Mobility Management :car:
### The Chassis

### Drive System

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

