# Vision-Guided Mecanum Rover

A Raspberry Pi 5 based autonomous and teleoperated mecanum wheel robot with computer vision target tracking.

---

## 📌 Overview

This project is a four-wheel mecanum robot capable of:

- Manual control via game controller (teleoperation)
- Serial communication with Arduino Mega
- Real-time camera processing using OpenCV
- Autonomous red object tracking
- Omnidirectional movement (x, y, θ control)

---

## 🧠 System Architecture

Raspberry Pi 5
→ Python (OpenCV + Control Logic)
→ Serial Communication
→ Arduino Mega
→ Motor Drivers
→ Mecanum Wheel Platform

Camera
→ OpenCV
→ HSV Color Detection
→ Target Center Extraction
→ Motion Control (vx, vy, wz)

---

## 🛠 Hardware

- Raspberry Pi 5
- Arduino Mega
- USB Camera
- Mecanum Wheel Chassis
- Game Controller
- Motor Driver (4 channels)

---

## 🎮 Control Modes

### Manual Mode
- Controlled via joystick
- Axis mapping:
  - Left stick → vx / vy
  - Right stick → rotation (wz)

### Autonomous Mode
- Detects red colored target
- Computes centroid error
- Converts error → angular velocity (wz)
- Moves toward target automatically

---

## 📷 Computer Vision

- HSV color segmentation
- Contour detection
- Largest contour selection
- Bounding box + centroid extraction

---

## 🔁 Communication

Format:
