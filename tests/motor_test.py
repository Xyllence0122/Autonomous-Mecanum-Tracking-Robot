#!/usr/bin/env python3
"""Low-speed motor direction test. Keep wheels lifted off the ground."""

import argparse
from pathlib import Path
import sys
import time

sys.path.append(str(Path(__file__).resolve().parents[1] / "raspberry_pi"))

from serial_comm import MotorSerial, SerialConfig  # noqa: E402


TESTS = [
    ("LF forward", (80, 0, 0, 0)),
    ("RF forward", (0, 80, 0, 0)),
    ("LR forward", (0, 0, 80, 0)),
    ("RR forward", (0, 0, 0, 80)),
    ("Forward", (80, 80, 80, 80)),
    ("Backward", (-80, -80, -80, -80)),
    ("Turn right", (80, -80, 80, -80)),
    ("Turn left", (-80, 80, -80, 80)),
]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--serial-port", default="/dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    motor = MotorSerial(SerialConfig(args.serial_port, args.baudrate, dry_run=args.dry_run))
    motor.open()

    try:
        for name, wheels in TESTS:
            input(f"\nLift wheels. Press Enter to run: {name}")
            motor.send(wheels)
            time.sleep(args.duration)
            motor.stop()
            time.sleep(0.5)
    finally:
        motor.stop()
        motor.close()


if __name__ == "__main__":
    main()

