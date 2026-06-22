# Autonomous Mecanum Tracking Robot

## Overview

Autonomous Mecanum Tracking Robot is a computer vision-based mobile robot developed using a Raspberry Pi 5 and Arduino Mega 2560.

The system detects and tracks a red target using OpenCV image processing techniques and autonomously drives a four-wheel mecanum platform through real-time visual feedback.

The project demonstrates the integration of:

* Computer Vision
* Mobile Robotics
* Mecanum Wheel Kinematics
* Embedded Systems
* Human Teleoperation
* Autonomous Target Tracking

---

## Hardware Components

| Component        | Model                    |
| ---------------- | ------------------------ |
| SBC              | Raspberry Pi 5           |
| Operating System | Ubuntu 24.04 LTS         |
| Microcontroller  | Arduino Mega 2560        |
| Camera           | USB Camera               |
| Drive System     | 4-Wheel Mecanum Platform |
| Communication    | USB Serial (115200 baud) |

---

## System Architecture

```text
USB Camera
    │
    ▼
Raspberry Pi 5
(Ubuntu 24.04)
    │
    ▼
OpenCV Vision Processing
    │
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
USB Serial Communication
    │
    ▼
Arduino Mega 2560
    │
    ▼
Motor Drivers
    │
    ▼
4-Wheel Mecanum Robot
```

---

## Repository Structure

```text
Autonomous-Mecanum-Tracking-Robot
│
├── README.md
├── requirements.txt
├── LICENSE
│
├── docs
│   ├── system_architecture.png
│   ├── wiring_diagram.png
│   └── demo_video_link.txt
│
├── arduino
│   └── mecanum_controller.ino
│
├── raspberry_pi
│   ├── main_teleop.py
│   ├── red_tracking.py
│   ├── serial_comm.py
│   └── vision_control.py
│
├── calibration
│   ├── hsv_tuner.py
│   └── joystick_mapper.py
│
└── tests
    ├── motor_test.py
    ├── serial_test.py
    └── camera_test.py
```

---

## Features

### Manual Teleoperation

* USB Game Controller Support
* Omnidirectional Mecanum Movement
* Real-Time Wheel Velocity Control

### Autonomous Tracking

* HSV Color Segmentation
* Red Target Detection
* Contour Filtering
* Target Locking Mechanism
* Distance-Based Motion Control
* Automatic Center Alignment

### Safety Features

* Automatic Stop When Target Is Lost
* Wheel Speed Limiting
* Communication Failure Protection

---

## Installation

Clone the repository:

```bash
git clone https://github.com/Xyllence0122/Autonomous-Mecanum-Tracking-Robot.git
cd Autonomous-Mecanum-Tracking-Robot
```

Create a Python virtual environment:

```bash
python3 -m venv venv
source venv/bin/activate
```

Install dependencies:

```bash
pip install -r requirements.txt
```

---

## Running the System

Activate the virtual environment:

```bash
source venv/bin/activate
```

Manual driving mode:

```bash
python raspberry_pi/main_teleop.py
```

Autonomous tracking mode:

```bash
python raspberry_pi/vision_control.py
```

---

## Mecanum Wheel Mixing

The robot uses standard mecanum inverse kinematics:

```text
LF = Vy + Vx + Wz

RF = Vy - Vx - Wz

LR = Vy - Vx + Wz

RR = Vy + Vx - Wz
```

Where:

* Vx = Lateral Velocity
* Vy = Forward Velocity
* Wz = Rotational Velocity

---

## Computer Vision Pipeline

1. Capture image from USB Camera
2. Convert BGR image to HSV
3. Apply red color threshold
4. Perform morphological filtering
5. Extract contours
6. Score candidate targets
7. Lock onto the best target
8. Estimate target position
9. Generate wheel velocity commands

---

## Current Project Status

### Completed

* Raspberry Pi 5 ↔ Arduino Mega Serial Communication
* USB Game Controller Teleoperation
* Four-Wheel Mecanum Drive Control
* OpenCV Red Target Detection
* HSV-Based Color Segmentation
* Autonomous Target Tracking
* Target Locking and Filtering
* Real-Time Wheel Velocity Control

### In Progress

* Software Modularization
* Manual / Autonomous Mode Switching
* Logging and Diagnostics System

### Planned

* YOLO Object Detection
* ROS 2 Integration
* Multi-Target Tracking
* Autonomous Navigation
* Web Monitoring Dashboard

---

## Future Improvements

* YOLO-Based Object Detection
* ROS 2 Migration
* SLAM Navigation
* Web Dashboard
* PID Auto-Tuning

---

## Demonstration

Add project videos, GIFs, or images here.

```text
docs/demo_video_link.txt
```

---

## License

This project is licensed under the GNU General Public License v3.0 (GPL-3.0).

See the LICENSE file for details.

---

## Author

Chao-Lin Chen

Department of Intelligent Automation Engineering

National Taipei University of Technology

Taipei, Taiwan

---

## Project Goal

The goal of this project is to develop an autonomous vision-guided mecanum robot capable of detecting, tracking, and approaching a target object in real time.

The system combines computer vision, embedded control, and omnidirectional mobile robotics into a lightweight and low-cost platform suitable for education, research, and robotics competitions.
