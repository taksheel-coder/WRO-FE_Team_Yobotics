Future Engineers 2025 – YoBotics



**Table of Contents**


-Overview

-Repository Structure

-Team Members

-Hardware/Components

-Mobility System

-Power & Sensors

-Software

-Setup & Initialization

-Object Detection & Navigation

-Driving Strategies & Challenge Logic

-Assembly Instructions

-Potential Improvements

-License / Acknowledgements



**Overview**

This project is an autonomous vehicle designed for the WRO Future Engineers 2025 competition, built to complete both the Open Challenge and the Obstacle Challenge.

It serves as both a competition-ready bot and a reference platform for exploring navigation strategies, wall-following logic, and obstacle detection.



**Key features:**

Front-wheel steering and rear-wheel drive design for realistic vehicle dynamics

Raspberry Pi as the main processing unit for navigation and control

Wide-angle camera for colour-based detection and obstacle recognition

Ultrasonic sensor for precise distance measurement and wall following

IMU-based orientation tracking for accurate lap counting and stable turns

Modular software for quick adaptation to new strategies



**Repository Structure**

| Directory      | Description                           |
|----------------|---------------------------------------|
| Arduino_Files  | Arduino-related test files and references |
| Schematics     | Wiring diagrams and electronic layouts |
| Team Photos    | Reserved for team member photos        |
| Vehicle Photos | Images of robot builds and prototypes  |
| Video          | Demo and challenge run videos          |
| cad            | 3D CAD models for chassis and components |
| src            | Source code for the Raspberry Pi controller |



**Team Members**

Team Name: YoBotics

Members:

Ayush Kothari – ayushkothari2007@gmail.com

Taksheel Subudhi – taksheelsubudhi@gmail.com

Aneesh Vijay - aneeshvijay41@gmail.com



**Hardware/Components**

| Component         | Description                 | Notes |
|-------------------|-----------------------------|-------|
| Chassis           | Custom 3D printed frame     | Optimized for turning radius, weight distribution, and colour detection |
| DC Motor          | 100 RPM Dual Shaft BO Motor | Rear-wheel drive |
| Motor Driver      | L298N                       | Connected to Raspberry Pi |
| Steering Servo    | MG996R high-torque servo    | Front-wheel steering |
| Raspberry Pi      | Main processing unit        | Runs navigation & control |
| Wide-Angle Camera | Vision input                | For colour and obstacle detection |
| Ultrasonic Sensor | Distance measurement        | For wall detection |
| Colour Sensor     | Line and turn detection     | Used in decision-making |
| MPU6050           | IMU                         | Provides orientation data |
| Battery           | 3.7V 2000mAh 18650 Li-Ion   | Powers electronics & motors |
| Misc.             | 3D-printed mounts, wiring   | — |



**Mobility System**

Configuration: Front-wheel steering with single dual-shaft BO motor for rear drive

Turning Radius: Optimized for narrow track corners

Control: PWM-based motor speed control, servo-based steering

Reasoning: Car-like steering dynamics with stability for WRO challenge field



**Power & Sensors**

Power: 3.7V 2000mAh 18650 Li-Ion battery


Sensors:

Ultrasonic + Colour Sensor: Wall following & turn detection

MPU6050 IMU: Orientation & lap tracking

Camera: Obstacle and colour detection



**Software**


**Setup & Initialization**

Raspberry Pi

Install Raspberry Pi OS

Install dependencies as required



**Motor & Sensor Setup**

Use /src/ files on Raspberry Pi

Run test scripts to verify motor, servo, and sensor calibration



**Object Detection & Navigation**

Ultrasonic logic for wall detection and alignment

Colour sensor detects line colour for turn direction

Camera detects obstacles via colour recognition

Fusion of IMU and servo angle ensures accurate turning



**Driving Strategies & Challenge Logic**

Wall Following: Ultrasonic + colour sensor feedback

Turns: Servo steering with colour sensor detection

Obstacle Avoidance: Camera-based rerouting with servo control

Lap Counting: IMU orientation and angle tracking

Recovery: Re-alignment using ultrasonic + camera data



**Assembly Instructions**

Mount DC motor and servo securely on chassis

Calibrate servo steering angles

Install Raspberry Pi on vibration-dampened mounts

Wire battery, motor driver, sensors, and camera carefully

Run test scripts for motors, servo, and sensors before full runs



**Potential Improvements**

Improve reliability of detection algorithms

Refine calibration of colour sensor and camera angle

Explore reinforcement learning for adaptive navigation



**License / Acknowledgements**

This project is licensed under the MIT License.

Special thanks to:

OpenCV – Computer vision processing

Raspberry Pi Foundation – SBC platform

WRO community and participating teams for continuous inspiration

