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

## Furitek Micro Komodo 1212 3450KV Brushless Motor
We selected this motor due to its lightweight design and high power. The motor is high effeciency, and allows control over the speed of the car. This motor allows us to frequently change the speed and acceleration of the car, giving a high degree of control over the car. The motor's teeth pair with the GT24 Differential gear set, eliminating the need for any gear in between.

## SunFounder SG90 Mirco Digital 9G Servo
Steering precision is of the upmost importance for this competition. This servo gives us sufficient torque for turning the wheels of our vehicle with great accuracy, granting us control over the direction of the car. The servo is also supported by the raspberry pi through the PWM control, allowing for quick adjustments and accurate turns with little delay. 

# Power Considerations
The GEns Ace 2S1p 1300mAh 7.4 LiPo battery is overkill for the power supply of our car. This battery has a continous discharge rate of 25C, delivering more than enough power for the peak demand of our electronics, inlcuding the SG90 Servo, Furitek motor, ESC, Raspbeery Pi 5 and the RRC Lite Controller. The ESC, connected directly to the controller, regulates the voltage to the motor, avoiding brownouts even under full load.

# Power Draw
| Component | Voltage | Max Current Draw |
| RRC Lite Controller | 5v | 0.5A |
| YDLidar T-mini|5v|0.3A|
| Furitek Micro Komodo Motor | 7.4V | 5A |
| Furitek Lizard Pro ESC | 8.4V | 50A |
| SG90 Servo Motor | 5v | 0.25A |
| Pi Camera | 5v | 0.25A |
| MicroSD, Leds, and Speakers | 5v | 0.2A |
 Total Power Draw
| Category | Value |
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
