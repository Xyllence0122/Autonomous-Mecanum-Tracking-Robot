# Raspberry Pi 5 Red Target Tracking Mecanum Robot

This project uses OpenCV HSV color segmentation and contour detection to track a red corrugated board. Wheel speed commands are transmitted to an Arduino Mega through USB Serial communication for controlling a four-wheel mecanum robot.

---

## Installation

Install the required Python packages:

```bash
python3 -m pip install -r requirements.txt
```

If you are using Raspberry Pi OS and prefer system-managed packages, you can install the dependencies with:

```bash
sudo apt update
sudo apt install -y python3-opencv python3-serial python3-numpy
```

---

## Running the Program

First, identify the serial port connected to the Arduino Mega:

```bash
ls /dev/ttyACM* /dev/ttyUSB*
```

### Standard Execution

```bash
python3 red_mecanum_tracker.py --serial-port /dev/ttyACM0
```

### Camera and Detection Test Only (No Arduino Output)

```bash
python3 red_mecanum_tracker.py --dry-run
```

### Headless Mode (Without GUI Display)

```bash
python3 red_mecanum_tracker.py --serial-port /dev/ttyACM0 --no-display
```

---

## Common Parameter Adjustments

```bash
python3 red_mecanum_tracker.py \
  --serial-port /dev/ttyACM0 \
  --kp 0.003 \
  --target-area 18000 \
  --min-area 1000 \
  --forward-speed 0.35 \
  --max-wz 0.65 \
  --center-deadband-px 25
```

### Parameter Description

| Parameter              | Description                                                                                                                                                         |
| ---------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `--kp`                 | Proportional gain for converting horizontal tracking error into rotational velocity. If too large, the robot may oscillate; if too small, turning will be sluggish. |
| `--target-area`        | Target contour area used to determine when the robot should stop approaching the object. Larger values cause the robot to stop earlier.                             |
| `--min-area`           | Minimum contour area threshold used to filter out noise and small objects.                                                                                          |
| `--forward-speed`      | Forward velocity while approaching the target. A recommended starting range is 0.20–0.35.                                                                           |
| `--max-wz`             | Maximum rotational control output used to limit turning speed.                                                                                                      |
| `--center-deadband-px` | Deadband around the image center to reduce oscillation when the target is nearly centered.                                                                          |

---

## Direction Control

The tracking controller uses the following control equation:

```python
error_x = cx - target_x
wz = -error_x * Kp
```

If the robot rotates in the opposite direction from what is expected, try using a negative proportional gain:

```bash
python3 red_mecanum_tracker.py --serial-port /dev/ttyACM0 --kp -0.003
```

---

## Serial Communication Format

The Arduino Mega receives wheel commands in the following format:

```text
LF,RF,LR,RR\n
```

Example:

```text
120,-80,120,-80
```

Wheel speed values are constrained to the range:

```text
-255 ~ 255
```

where:

* **LF** = Left Front Wheel
* **RF** = Right Front Wheel
* **LR** = Left Rear Wheel
* **RR** = Right Rear Wheel

---

## System Overview

**Raspberry Pi 5**

* Captures images from a USB camera
* Detects and tracks a red target using OpenCV
* Computes mecanum wheel velocity commands
* Sends wheel speeds to Arduino Mega via USB Serial

**Arduino Mega**

* Receives wheel commands
* Controls the motor drivers
* Drives the four mecanum wheels

This architecture enables autonomous visual target tracking while maintaining a lightweight and modular hardware design.
