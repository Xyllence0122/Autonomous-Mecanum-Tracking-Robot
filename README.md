# Autonomous Mecanum Tracking Robot

## Overview

Autonomous Mecanum Tracking Robot is a computer vision-based mobile robot developed using a Raspberry Pi 5 and Arduino Mega 2560.

The system detects and tracks a red target using OpenCV image processing techniques and autonomously drives a four-wheel mecanum platform through real-time visual feedback.

The project demonstrates the integration of:

- Computer Vision
- Mobile Robotics
- Mecanum Wheel Kinematics
- Embedded Systems
- Human Teleoperation
- Autonomous Target Tracking

---

## System Architecture

USB Camera
     │
     ▼
Raspberry Pi 5
(Ubuntu 24.04)
     │
     │ OpenCV
     ▼
Target Detection
     │
     ▼
Tracking Controller
     │
     ▼
Mecanum Kinematics
     │
     ▼
USB Serial
     │
     ▼
Arduino Mega 2560
     │
     ▼
Motor Drivers
     │
     ▼
4 Mecanum Wheels

---

## Hardware Components

| Component | Model |
|------------|---------|
| SBC | Raspberry Pi 5 |
| Operating System | Ubuntu 24.04 LTS |
| Microcontroller | Arduino Mega 2560 |
| Camera | USB Camera |
| Drive System | 4-Wheel Mecanum Platform |
| Communication | USB Serial (115200 baud) |

---

## Features

### Manual Teleoperation

- USB Game Controller Support
- Omnidirectional Mecanum Movement
- Real-Time Wheel Velocity Control

### Autonomous Tracking

- HSV Color Segmentation
- Red Object Detection
- Contour Filtering
- Target Locking Mechanism
- Distance-Based Motion Control
- Automatic Center Alignment

### Safety Functions

- Target Lost Detection
- Automatic Stop
- Wheel Speed Limiting
- Communication Timeout Protection

---

## Repository Structure

Autonomous-Mecanum-Tracking-Robot
│
├── README.md
├── requirements.txt
├── LICENSE
│
├── docs
│   ├── system_architecture.png
│   ├── wiring_diagram.png
│   └── demo_video.gif
│
├── arduino
│   └── mecanum_controller.ino
│
├── raspberry_pi
│   ├── main_tracking.py
│   ├── teleop_controller.py
│   ├── vision.py
│   ├── tracker.py
│   ├── controller.py
│   └── serial_comm.py
│
├── calibration
│   ├── hsv_tuner.py
│   └── joystick_mapper.py
│
└── tests
    ├── motor_test.py
    ├── serial_test.py
    └── camera_test.py

---

## Software Environment

Operating System

Ubuntu 24.04 LTS

Python Version

Python 3.12+

Dependencies

pip install -r requirements.txt

requirements.txt

opencv-python
numpy
pyserial
pygame

---

## Arduino Setup

Upload:

arduino/mecanum_controller.ino

to:

Arduino Mega 2560

Baudrate:

Serial.begin(115200);

---

## Running the System

Activate Python Environment

cd ~/Robot
source venv/bin/activate

Manual Driving Mode

python raspberry_pi/teleop_controller.py

Autonomous Tracking Mode

python raspberry_pi/main_tracking.py

---

## Mecanum Wheel Mixing

Wheel velocities are generated using standard mecanum inverse kinematics:

LF = Vy + Vx + Wz

RF = Vy - Vx - Wz

LR = Vy - Vx + Wz

RR = Vy + Vx - Wz

Where:

Vx = Lateral Velocity

Vy = Forward Velocity

Wz = Rotational Velocity

---

## Computer Vision Pipeline

1. Capture image from USB Camera

2. Convert BGR to HSV

3. Apply red color threshold

4. Morphological filtering

5. Contour extraction

6. Candidate scoring

7. Target locking

8. Position estimation

9. Motion command generation

---

## Future Improvements

- YOLO Object Detection
- Multi-Target Tracking
- ROS 2 Integration
- SLAM Navigation
- Web-Based Monitoring Dashboard
- PID Auto-Tuning

---

## Demonstration

Add demonstration images or videos here.

docs/demo_video.gif

---

## License

This project is licensed under the GNU General Public License v3.0 (GPL-3.0).

You are free to use, modify, and distribute this software under the terms of the GPL-3.0 license.

Any derivative work based on this project must also be released under the GPL-3.0 license and provide access to the corresponding source code.

For more details, see the LICENSE file included in this repository.

---

## Author

Chao-Lin Chen

Department of Intelligent Automation of Engineering

National Taipei University of Technology

Taiwan

---

## Project Goal

The primary objective of this project is to develop an autonomous vision-guided mecanum robot capable of detecting, tracking, and approaching a target object in real time.

The system combines computer vision, embedded control, and omnidirectional mobile robotics into a lightweight and low-cost platform suitable for educational, research, and robotics competition applications.

---

## Current Project Status

Completed:

- Raspberry Pi 5 and Arduino Mega serial communication
- USB game controller teleoperation
- Four-wheel mecanum drive control
- OpenCV red target detection
- HSV-based color segmentation
- Autonomous target tracking
- Target locking and filtering
- Real-time wheel velocity control

In Progress:

- Software modularization
- Automatic/manual mode switching
- System logging and diagnostics

Planned:

- YOLO-based object detection
- ROS 2 migration
- Multi-object tracking
- Autonomous navigation
- Web monitoring interface