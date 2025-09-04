# Future Engineers 2025 – YoBotics

**Table of Contents**
- Overview
- System Architecture
- Repository Structure
- Team Members
- Hardware & Components
- Mobility & Power System
- Sensors & Vision
- Software Overview
- Setup & Initialization
- Object Detection & Navigation
- Driving Strategies & Challenge Logic
- Testing & Calibration
- Results & Performance
- Troubleshooting Guide
- Assembly Instructions
- Potential Improvements
- License / Acknowledgements

---

## Overview

YoBotics is an autonomous vehicle engineered for the WRO Future Engineers 2025 competition, designed to tackle both the Open and Obstacle Challenges. The bot demonstrates advanced navigation, wall-following, and obstacle avoidance using modular hardware and software.

---

## System Architecture

![System Architecture Diagram](insert_diagram_here)

The robot integrates a Raspberry Pi for centralized control, a Logitech camera for vision, and a dual power system. All sensors and actuators are connected through a custom wiring harness, enabling robust data flow and control.

---

## Repository Structure

| Directory         | Description                                 |
|-------------------|---------------------------------------------|
| Arduino_Files     | Arduino test files and references           |
| Schematics        | Wiring diagrams and electronic layouts       |
| Team Photos       | Team member photos                          |
| Vehicle Photos    | Robot builds and prototypes                 |
| Video             | Demo and challenge run videos               |
| cad               | 3D CAD models for chassis and components    |
| src               | Raspberry Pi controller source code         |
| docs              | Detailed guides and troubleshooting         |

---

## Team Members

Team Name: YoBotics  
- Ayush Kothari – ayushkothari2007@gmail.com  
- Taksheel Subudhi – taksheelsubudhi@gmail.com  
- Aneesh Vijay – aneeshvijay41@gmail.com  

---

## Hardware & Components

| Component         | Description                                                                 | Notes                                         |
|-------------------|-----------------------------------------------------------------------------|-----------------------------------------------|
| Chassis           | Custom 3D printed frame                                                     | Optimized for turning radius and weight       |
| DC Motor          | 100 RPM Dual Shaft BO Motor (Rear drive)                                    | Stable rear-wheel drive                       |
| Steering Servo    | MG996R high-torque servo (Ackerman steering)                                | Precise front-wheel steering                  |
| Raspberry Pi      | Main processing unit                                                        | Runs navigation & control                     |
| Logitech Camera   | 92mm focal length                                                           | Colour and obstacle detection                 |
| Ultrasonic Sensor | Distance measurement                                                        | Wall following and alignment                  |
| Colour Sensor     | Line and turn detection                                                     | Decision-making for turns                     |
| MPU6050 IMU       | Orientation and lap tracking                                                | Accurate lap count and stable turns           |
| Bonka LiPo Battery| 12V, powers motors and electronics                                          | High-capacity for extended runs               |
| Mi Power Bank     | 10,000mAh, powers Raspberry Pi                                              | Isolated Pi power for reliability             |
| Misc.             | 3D-printed mounts, wiring                                                   | Vibration dampening and cable management      |

---

## Mobility & Power System

- **Drive:** Rear-wheel drive via dual shaft BO motor
- **Steering:** Ackerman geometry using MG996R servo
- **Power:** 12V Bonka LiPo battery for motors, Mi 10,000mAh power bank for Raspberry Pi

---

## Sensors & Vision

- **Camera:** Logitech USB camera, 92mm focal length, mounted for wide-angle vision
- **Ultrasonic Sensor:** For wall detection and distance measurement
- **Colour Sensor:** Detects line colour for navigation and turns
- **IMU (MPU6050):** Tracks orientation and lap count

---

## Software Overview

- Modular Python scripts for navigation, sensor fusion, and challenge logic
- Source code organized in `/src` folder
- OpenCV for vision processing
- Easy configuration for new strategies

---

## Setup & Initialization

1. Install Raspberry Pi OS and required dependencies
2. Connect all sensors and actuators as per schematics
3. Calibrate motors, servo, and sensors using test scripts
4. Configure camera settings for optimal colour detection

---

## Object Detection & Navigation

- The object detection system is primarily camera-based. The Logitech camera processes images using OpenCV, applying a broad HSV color range to identify blocks, but combines this with strict shape analysis to avoid false positives.
- For wall detection, a dedicated HSV mask isolates the wall’s color signature, ensuring only the wall is detected in the field of view.
- Accurate distance measurement is achieved by leveraging the camera’s 92mm focal length and mathematical calculations with NumPy, allowing the robot to estimate the position of objects and walls for precise navigation.

---
## Driving Strategies & Challenge Logic
## Driving Strategies & Challenge Logic

- **No Obstacle Round:**  
  In rounds without obstacles, the robot relies primarily on its camera for navigation. The motors are set to run at a steady, constant speed, allowing the bot to move forward smoothly. The navigation algorithm uses a Region of Interest (ROI) within the camera feed to continuously scan for walls.  
  - If a wall is detected directly ahead, the bot evaluates the presence of walls on either side. If a side wall is detected, it immediately executes a sharp turn away from that wall using a predefined steering value, ensuring a quick and decisive maneuver to avoid collision.
  - If only a front wall is detected and there are no side walls, the bot initiates a weaving motion—alternately steering left and right—to explore potential open paths and avoid getting stuck.
  - This wall-sensing and avoidance logic is repeated throughout the round, allowing the robot to dynamically adapt to changes in wall distance, orientation, and track layout. The approach ensures continuous forward movement while maintaining a safe distance from all obstacles.

- **Obstacle Round:**  
  When the robot encounters an obstacle (such as a block), it uses its camera to center itself precisely with respect to the obstacle, ensuring accurate alignment. The robot then continues to move forward until it reaches a predetermined distance from the obstacle, calculated using camera data and onboard algorithms.  
  - At this point, the robot determines the direction it needs to turn based on the detected color of the block, allowing for color-based decision making and challenge compliance.
  - After successfully navigating past the obstacle, the robot seamlessly transitions back to the no obstacle navigation logic, resuming its forward movement and wall detection routines.
  - To enhance precision and safety during obstacle negotiation, the robot automatically reduces its speed to 60% of its normal value when a block is detected. This speed reduction allows for more controlled and accurate maneuvers, minimizing the risk of errors or collisions while passing obstacles.
---

## Testing & Calibration

- Step-by-step guides for calibrating each subsystem
- Example scripts and troubleshooting tips in `/docs`
- Photos and videos of test runs included in `/Video` and `/Vehicle Photos`

---

## Results & Performance

| Test            | Result               |
|-----------------|---------------------|
| Lap Time        | 00:XX (sample)      |
| Detection Accuracy | XX% (sample)     |
| Power Runtime   | XX minutes (sample) |

Videos and images available in the repository.

---

## Troubleshooting Guide

- Common issues and solutions for sensors, motors, and software
- FAQ for setup and calibration

---

## Assembly Instructions

1. Mount DC motor and servo securely on chassis
2. Install Raspberry Pi on vibration-dampened mounts
3. Wire battery, motor driver, and camera
4. Run test scripts before full challenge runs

---
## Potential Improvements

- **Enhance Detection Algorithms for Reliability:**  
  Refine the computer vision pipeline to minimize false positives and false negatives in both block and wall detection. Consider implementing adaptive thresholding, advanced contour filtering, or even lightweight machine learning models to improve detection accuracy under varying lighting and field conditions.

- **Refine Sensor Calibration and Camera Angle:**  
  Develop systematic calibration routines for the color sensor, camera, and IMU. Document the optimal mounting angles and positions for all sensors, and include procedures to repeat calibration before each competition run to ensure consistent performance.

- **Explore Reinforcement Learning for Adaptive Navigation:**  
  Investigate the use of reinforcement learning algorithms to enable the robot to adapt its navigation strategy based on real-time feedback. By training the bot in simulation or controlled environments, it could learn to optimize wall-following, turning, and obstacle avoidance dynamically.

- **Integrate Data Logging and Analysis:**  
  Implement onboard data logging for sensor readings and decision events. Analyzing this data post-run can help identify bottlenecks, unexpected behaviors, and areas for further optimization.

- **Improve Power Management:**  
  Explore advanced power regulation modules to ensure stable voltage supply, especially when motors and processing units are under heavy load, to prevent unexpected resets or lags.

- **Expand Documentation and User Guides:**  
  Add detailed assembly guides, troubleshooting flowcharts, and a comprehensive FAQ section. Include video tutorials and annotated wiring diagrams to make it easier for new users or team members to understand and replicate the build.
---

## License / Acknowledgements

Project licensed under MIT License.

Special thanks to:
- OpenCV (computer vision)
- Raspberry Pi Foundation (SBC platform)
- WRO community and participating teams

---

For more details, see `/docs` and example videos in `/Video`.
