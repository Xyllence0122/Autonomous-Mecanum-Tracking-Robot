#!/usr/bin/env python3
"""Serial communication wrapper for Arduino Mega motor commands."""

import time
from dataclasses import dataclass
from typing import Optional, Tuple

import serial


WheelSpeeds = Tuple[int, int, int, int]


@dataclass
class SerialConfig:
    port: str = "/dev/ttyUSB0"
    baudrate: int = 115200
    timeout: float = 1.0
    reset_wait: float = 2.0
    dry_run: bool = False


class MotorSerial:
    def __init__(self, config: SerialConfig):
        self.config = config
        self.ser: Optional[serial.Serial] = None

    def open(self) -> None:
        if self.config.dry_run:
            print("[serial] dry-run mode enabled")
            return

        self.ser = serial.Serial(
            self.config.port,
            self.config.baudrate,
            timeout=self.config.timeout,
        )
        time.sleep(self.config.reset_wait)
        print(f"[serial] connected to {self.config.port} @ {self.config.baudrate}")

    def close(self) -> None:
        if self.ser is not None:
            self.ser.close()
            self.ser = None

    def send(self, wheels: WheelSpeeds) -> None:
        lf, rf, lr, rr = wheels
        msg = f"{lf},{rf},{lr},{rr}\n"

        if self.config.dry_run:
            print(msg.strip())
            return

        if self.ser is None:
            raise RuntimeError("serial port is not open")

        self.ser.write(msg.encode("ascii"))

    def stop(self) -> None:
        self.send((0, 0, 0, 0))


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def clamp_speed(value: float) -> int:
    return int(clamp(value, -255, 255))


def mecanum_mix(vx: float, vy: float, wz: float, scale: int = 150) -> WheelSpeeds:
    """Mecanum mixing formula verified by the joystick baseline."""
    lf = int((vy + vx + wz) * scale)
    rf = int((vy - vx - wz) * scale)
    lr = int((vy - vx + wz) * scale)
    rr = int((vy + vx - wz) * scale)

    return (
        clamp_speed(lf),
        clamp_speed(rf),
        clamp_speed(lr),
        clamp_speed(rr),
    )

