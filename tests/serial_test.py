#!/usr/bin/env python3
"""Serial protocol test. Sends stop commands to Arduino."""

import argparse
from pathlib import Path
import sys
import time

sys.path.append(str(Path(__file__).resolve().parents[1] / "raspberry_pi"))

from serial_comm import MotorSerial, SerialConfig  # noqa: E402


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--serial-port", default="/dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    motor = MotorSerial(SerialConfig(args.serial_port, args.baudrate, dry_run=args.dry_run))
    motor.open()
    try:
        for _ in range(20):
            motor.stop()
            time.sleep(0.05)
        print("Serial stop-command test completed.")
    finally:
        motor.close()


if __name__ == "__main__":
    main()

