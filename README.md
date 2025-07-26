## Engineering Materials
* Raspberry Pi 5 (8 GB)
* [RRC (Ros Robot Controller) Lite Controller](https://www.hiwonder.com/products/rrc-lite?srsltid=AfmBOorP3iSszjlniFgA0gsbfA9aUH3UK0MWuyPeuTUcW6RMQjcizhmE)
* [GT24 B Differential Gear Set from Carisma RC](https://carisma-shop.com/collections/gt24-spares/products/gt24-b-differential-gear-set)
* [Furitek Micro Komodo 1212 3450KV Brushless Motor](https://furitek.com/products/furitek-micro-komodo-1212-3456kv-brushless-motor-with-15t-steel-pinion-for-fury-wagon-fx118)
* [Furiteck Lizard Pro 30A/50A Brushless ESC](https://furitek.com/products/combo-of-furitek-lizard-pro-30a-50a-brushed-brushless-esc-for-axial-scx24-with-bluetooth)
* [Gens Ace 2S1P 1300mAh 7.4V battery](https://www.adrenalinehobby.com/products/gens-ace-g-tech-1300mah-2s-7-4v-25c-lipo-deans-plug?_pos=1&_sid=dde29d30b&_ss=r)
* [Sun Founder SG90 Micro Digital 9G Servo Motor](https://www.sunfounder.com/products/sg90-micro-digital-servo?srsltid=AfmBOop4G8SB4zvimDdmDlNUaAaMoN5-eXqEeMZD69HXEi-1QH7Qkzmw)
* Mini Rocker Switch
* [Raspberry 5 Camera Module 8 MP MIPI-CSI Interface](https://www.amazon.ca/TUOPUONE-Compatible-Raspberry-MIPI-CSI-Interface/dp/B0CPTPJLXL?th=1)
* Micro SD card

# Movement Considerations
For the use of this competition, movement precision and responsiveness are the top priorities.

We selected the Furitek Micro Komodo 1212 3450KV Brushless Motor due to its lightweight design and high power. The motor is high efficiency and allows control over the speed of the car. This motor will enable us to frequently change the speed and acceleration of the car, giving a high degree of control over the car. The motor's teeth pair with the GT24 Differential gear set, eliminating the need for any gear in between. Brushless motors also have numerous benefits over brushed motors, offering an increased lifespan, less maintenance, and higher efficiency with better speed control.

This motor is connected to the Furiteck Lizard Pro 30A/50A Brushless ESC, enhancing the precise speed control and regulating the power directed to the motor. This combination gives the car smooth acceleration and deceleration, as well as ensures the safety of the motor.

Steering precision is of the utmost importance for this competition. The SunFounder SG90 Micro Digital 9G Servo gives us sufficient torque for turning the wheels of our vehicle with great accuracy, granting us control over the direction of the car. The servo is also supported by the Raspberry Pi through the PWM control, allowing for quick adjustments and accurate turns with little delay. This servo rotates a range of 180 degrees at 1.6kg of torque, giving more than enough power and distance to rotate the wheels of the car. This overkill torque ensures the vehicle can turn accurately at any speed, even when the car is not moving.

This car is based on a 1/24 sized RC car, small enough to control the car comfortably around any potential obstacles. This also makes it easier to maneuver in small spaces, such as the parking lot and pillars.

# Power Considerations
The Gens Ace 2S1p 1300mAh 7.4 LiPo battery is overkill for the power supply of our car. This battery has a continuous discharge rate of 45C, delivering more than enough power for the peak demand of our electronics, including the SG90 Servo, Furitek motor, ESC, Raspberry Pi 5 and the RRC Lite Controller. The ESC, connected directly to the controller, regulates the voltage to the motor, avoiding brownouts even under full load.

This battery is connected in parallel to the ESC and the Raspberry PI expansion board. The output of the battery and the power draw of various components are all listed below.

## Power Draw
| Component | Voltage | Max Current Draw |
| --- | --- | --- |
| RRC Lite Controller | 5v | 0.5A |
| YDLidar T-mini|5v|0.3A|
| Furitek Micro Komodo Motor | 7.4V | 5A |
| Furitek Lizard Pro ESC | 8.4V | 50A |
| SG90 Servo Motor | 5v | 0.25A |
| Pi Camera | 5v | 0.25A |
| MicroSD, Leds, and Speakers | 5v | 0.2A |

 Total Power Draw
| Category | Value |
|---|---|
| Average Current Draw | 10.5A |
| Battery Output Capacity | 32.5A |
| Headroom | 22A |

# Assembly Instructions



## Content

* `t-photos` contains 2 photos of the team (an official one and one funny photo with all team members)
* `v-photos` contains 6 photos of the vehicle (from every side, from top and bottom)
* `video` contains the video.md file with the link to a video where driving demonstration exists
* `schemes` contains one or several schematic diagrams in form of JPEG, PNG or PDF of the electromechanical components illustrating all the elements (electronic components and motors) used in the vehicle and how they connect to each other.
* `src` contains code of control software for all components which were programmed to participate in the competition
* `models` is for the files for models used by 3D printers, laser cutting machines and CNC machines to produce the vehicle elements. If there is nothing to add to this location, the directory can be removed.
* `other` is for other files which can be used to understand how to prepare the vehicle for the competition. It may include documentation how to connect to a SBC/SBM and upload files there, datasets, hardware specifications, communication protocols descriptions etc. If there is nothing to add to this location, the directory can be removed.

## Introduction

_This part must be filled by participants with the technical clarifications about the code: which modules the code consists of, how they are related to the electromechanical components of the vehicle, and what is the process to build/compile/upload the code to the vehicle’s controllers._

## How to prepare the repo based on the template

_Remove this section before the first commit to the repository_

1. Clone this repo by using the `git clone` functionality.
2. Remove `.git` directory
3. [Initialize a new public repository on GitHub](https://github.com/new) by following instructions from "create a new repository on the command line" section (appeared after pressing "Create repository" button).
