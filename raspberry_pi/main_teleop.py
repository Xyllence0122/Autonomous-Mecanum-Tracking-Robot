#!/usr/bin/env python3
"""Joystick teleoperation baseline."""

import argparse
import time

import pygame

from serial_comm import MotorSerial, SerialConfig, mecanum_mix


SCALE = 150
DEADZONE = 0.08


def dz(value: float) -> float:
    return 0.0 if abs(value) < DEADZONE else value


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--serial-port", default="/dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    motor = MotorSerial(SerialConfig(args.serial_port, args.baudrate, dry_run=args.dry_run))
    motor.open()

    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        raise RuntimeError("No joystick found")

    js = pygame.joystick.Joystick(0)
    js.init()

    try:
        while True:
            pygame.event.pump()

            lx = dz(js.get_axis(0))
            ly = dz(js.get_axis(1))
            rx = dz(js.get_axis(3))

            vx = lx
            vy = -ly
            wz = rx

            wheels = mecanum_mix(vx, vy, wz, SCALE)
            motor.send(wheels)
            print(f"{wheels[0]},{wheels[1]},{wheels[2]},{wheels[3]}", end="\r")
            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        motor.stop()
        motor.close()
        pygame.quit()


if __name__ == "__main__":
    main()

