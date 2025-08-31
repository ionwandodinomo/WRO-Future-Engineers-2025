
&nbsp;
Engineering Documentation 🛠️
======

<br>

> This repository details Team Tralalero Tralala’s building and programming process in the 2025 WRO Future Engineers Competition.

<br>

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

### Electrical Wiring


### The Sensors


<br>

## Software :computer:
### Initialization

### Object Detection

### Wall Management and Turning

<br>

## Assembly Instructions 🛠️

