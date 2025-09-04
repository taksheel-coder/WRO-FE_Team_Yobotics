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

YoBotics is an autonomous vehicle designed for the WRO Future Engineers 2025 competition, engineered to excel in both the Open and Obstacle Challenges. This project serves as a comprehensive platform for exploring advanced robotics concepts, including computer vision, sensor fusion, adaptive navigation, and modular hardware/software integration. Our goal is not only to compete at the highest level but also to provide a reference build for future teams and STEM learners.

---

## System Architecture

**System Block Diagram:**

```
+-------------------+          +-------------------+
|   Power Systems   |--------->|   Raspberry Pi    |
| (LiPo, Powerbank) |          +-------------------+
+-------------------+                  |
                                       |
            +--------------------------+--------------------------+
            |                          |                          |
   +----------------+        +-----------------+        +-----------------+
   |   Motors &     |        |     Sensors     |        |     Camera      |
   |   Servos       |        |     (IMU )      |        |  (Logitech USB) |
   | (BO, MG996R)   |        |                 |        +-----------------+
   +----------------+        +-----------------+
```

**Data Flow:**  
- Raspberry Pi acts as the main controller, running navigation, decision logic, and computer vision.
- Sensors and actuators are interfaced via GPIO, I2C, and USB.
- Power is delivered separately to the Pi and motors for stability.

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
| docs              | Detailed guides, flowcharts, troubleshooting|

---

## Team Members

Team Name: YoBotics

- Ayush Kothari – [ayushkothari2007@gmail.com](mailto:ayushkothari2007@gmail.com)
- Taksheel Subudhi – [taksheelsubudhi@gmail.com](mailto:taksheelsubudhi@gmail.com)
- Aneesh Vijay – [aneeshvijay41@gmail.com](mailto:aneeshvijay41@gmail.com)

---

## Hardware & Components

| Component         | Description                                                      | Notes                                         |
|-------------------|------------------------------------------------------------------|-----------------------------------------------|
| Chassis           | Custom 3D printed frame                                          | Optimized for turning radius and weight       |
| DC Motor          | 100 RPM Dual Shaft BO Motor (Rear drive)                         | Stable rear-wheel drive                       |
| Steering Servo    | MG996R high-torque servo (Ackerman steering)                     | Precise front-wheel steering                  |
| Raspberry Pi      | Main processing unit                                             | Runs navigation & control                     |
| Logitech Camera   | 92mm focal length, wide-angle                                   | Colour and obstacle detection                 |
| Ultrasonic Sensor | Distance measurement                                             | Wall following and alignment                  |
| Colour Sensor     | Line and turn detection                                          | Decision-making for turns                     |
| MPU6050 IMU       | Orientation and lap tracking                                     | Accurate lap count and stable turns           |
| Bonka LiPo Battery| 12V, powers motors and electronics                               | High-capacity for extended runs               |
| Mi Power Bank     | 10,000mAh, powers Raspberry Pi                                   | Isolated Pi power for reliability             |
| Misc.             | 3D-printed mounts, wiring                                        | Vibration dampening and cable management      |

---
Power and Sensor Management

Power management is built for stability and efficiency. The motors and main electronics are powered by a 12V Bonka LiPo battery, ensuring consistent performance during high-load operations. The Raspberry Pi is powered separately using a 10,000mAh Mi power bank, which isolates the control logic from motor surges and prevents unexpected resets. Additionally, a 5V buck converter is incorporated to supply regulated voltage to components that require lower power, further enhancing system reliability.

---

## Sensors & Vision

Sensor management is streamlined, relying exclusively on the Logitech USB camera as the primary sensing device. The camera is securely mounted on a vibration-dampened platform to maintain image clarity and accuracy. All wiring is shielded and routed for minimal interference, and calibration procedures for the camera are documented to ensure repeatable, high-quality object detection.
---

## Software Overview

- **Language:** Python for ease of development and integration with OpenCV and hardware libraries.
- **Structure:** Modular scripts for navigation, sensor fusion, challenge logic, and testing.
- **Vision:** OpenCV is used for real-time image processing, object detection, and ROI analysis.
- **Configuration:** All key parameters (HSV ranges, thresholds, motor speeds) are easily adjustable for quick tuning.

---

## Setup & Initialization

1. **Install Raspberry Pi OS** and update all packages.
2. **Clone repository** and install Python dependencies listed in `requirements.txt`.
3. **Connect all sensors and actuators** as shown in the wiring diagrams (`/Schematics`).
4. **Run calibration scripts** for motors, servo, camera, and sensors.
5. **Adjust camera settings** for optimal colour and shape detection.
6. **Test each subsystem** (motors, sensors, camera) individually before full integration.

---

## Object Detection & Navigation

**Flowchart: Object & Wall Detection**  
```
[Camera Frame] --> [HSV Mask for Blocks] --> [Shape Detection] --> [Block Found?]
                                                        |
                                                        v
                                             [If Yes: Center & Slow Down]
                                                        |
                                                        v
[HSV Mask for Wall] --> [Wall Found?] -----------No-----+-----Yes-----> [Initiate Wall Logic]
```

- The object detection system is primarily camera-based. The Logitech camera processes images using OpenCV, applying a broad HSV color range to identify blocks, but combines this with strict shape analysis to avoid false positives.
- For wall detection, a dedicated HSV mask isolates the wall’s color signature, ensuring only the wall is detected in the field of view.
- Accurate distance measurement is achieved by leveraging the camera’s 92mm focal length and mathematical calculations with NumPy, allowing the robot to estimate the position of objects and walls for precise navigation.

---

## Driving Strategies & Challenge Logic

**Flowchart: Driving Logic**  
```
[Start] --> [Obstacle Detected?] --No--> [Wall Detected?] --No--> [Drive Forward]
                                 |                         |
                                Yes                       Yes
                                 |                         |
                    [Center on Block & Slow Down]   [Check Side Walls]
                                 |                         |
                    [Move to Set Distance]         [Side Wall?]--Yes-->[Turn Opposite]
                                 |                         |         |
                    [Turn as per Block Color]      No      |         v
                                 |                [Weave to Find Path]
                    [Resume No-Obstacle Logic]
```

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

## Obstacle Management

Obstacle detection and handling are accomplished entirely through computer vision. The camera, using OpenCV, scans for blocks by applying a broad HSV color filter and strict shape analysis to distinguish valid obstacles from background noise. When an obstacle is detected, the robot centers itself with respect to the block and approaches at a reduced speed (40% lower than normal) for precise alignment and safe maneuvering.

The robot calculates the distance to the obstacle using the camera’s 92mm focal length and mathematical models implemented with NumPy. Upon reaching the target distance, it determines the correct turn direction based on the block’s color. After passing the obstacle, the robot resumes its wall-following and pathfinding logic, maintaining efficient progress throughout the challenge.

--- 

## Testing & Calibration

- **Step-by-step guides** for calibrating each subsystem (motors, servo, sensors, camera) are available in the `/docs` folder.
- **Test scripts** are provided for each hardware component to verify correct operation before integration.
- **Calibration routines** help ensure consistent sensor readings and reliable navigation.
- **Test run documentation** (photos and videos) is stored in `/Video` and `/Vehicle Photos` for reference and troubleshooting.

---

## Results & Performance

| Test                | Result               |
|---------------------|---------------------|
| Lap Time            | 00:XX (sample)      |
| Detection Accuracy  | XX% (sample)        |
| Power Runtime       | XX minutes (sample) |

- Videos and images of the robot in action are available in the repository.
- Performance metrics are updated after each test run for transparency.

---

## Troubleshooting Guide

- **Common issues and solutions** for sensors, motors, and software are documented in `/docs`.
- **FAQ section** covers frequent setup and calibration questions.
- **Diagnostic scripts** help isolate hardware or software faults quickly.
- **Error logs and data samples** are included for advanced debugging.

---

## Assembly Instructions

1. Mount DC motor and steering servo securely on the chassis using 3D-printed mounts.
2. Install the Raspberry Pi on vibration-dampened mounts to protect sensitive electronics.
3. Wire the battery, motor driver, sensors, and camera according to the provided schematics.
4. Double-check all connections for security and polarity.
5. Run individual test scripts for each subsystem before attempting a full system run.
6. Adjust sensor and camera angles as needed for optimal performance.

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

## Engineering Detail and Clarity

All engineering decisions and system designs are thoroughly documented for clarity and reproducibility. CAD models, wiring diagrams, and electronic schematics are provided in the repository. The README and ‎`/docs` folder offer in-depth descriptions of each subsystem, camera calibration procedures, and troubleshooting guides.

Software modules are organized by function—navigation, vision, and control logic—with all critical parameters exposed for tuning. Flowcharts and block diagrams illustrate the architecture and decision processes, making it easy for judges and new contributors to understand and evaluate the engineering work. The project emphasizes modularity, reliability, and transparency, ensuring every component and algorithm can be traced, tested, and improved.
---

## License / Acknowledgements

Project licensed under the MIT License.

Special thanks to:
- OpenCV (computer vision)
- Raspberry Pi Foundation (SBC platform)
- WRO community and participating teams
- All open-source contributors and documentation authors

---

For more details, see `/docs` and example videos in `/Video`.
