#!/usr/bin/env python3
"""Vision-guided red board following main program."""

import argparse
import time
from pathlib import Path
import sys
from typing import Optional

import cv2

sys.path.append(str(Path(__file__).resolve().parent))

from red_tracking import RedBoardTracker, RedTrackingConfig  # noqa: E402
from serial_comm import MotorSerial, SerialConfig, clamp, mecanum_mix  # noqa: E402


# Camera
CAMERA_INDEX = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# Distance control. Area inside this range means "close enough, stop".
STOP_AREA_MIN = 65000
STOP_AREA_MAX = 85000
AREA_FILTER_ALPHA = 0.20
CX_FILTER_ALPHA = 0.22

# Motion control
SCALE = 150
KP_TURN = 0.0018
CENTER_DEADBAND = 45
MAX_WZ = 0.35
WZ_FILTER_ALPHA = 0.25
MAX_WZ_STEP = 0.035
FORWARD_SPEED = 0.35
BACKWARD_SPEED = -0.22
ALIGN_FIRST_ERROR = 120
ROTATION_SIGN = 1
SEND_INTERVAL = 0.05


def low_pass(old_value: Optional[float], new_value: float, alpha: float) -> float:
    if old_value is None:
        return new_value
    return alpha * new_value + (1.0 - alpha) * old_value


def limit_step(new_value: float, old_value: float, max_step: float) -> float:
    delta = clamp(new_value - old_value, -max_step, max_step)
    return old_value + delta


def compute_vy(filtered_area: float, error_x: float) -> float:
    if abs(error_x) > ALIGN_FIRST_ERROR:
        return 0.0
    if filtered_area < STOP_AREA_MIN:
        return FORWARD_SPEED
    if filtered_area > STOP_AREA_MAX:
        return BACKWARD_SPEED
    return 0.0


def compute_wz(error_x: float, previous_wz: float) -> float:
    if abs(error_x) <= CENTER_DEADBAND:
        target_wz = 0.0
    else:
        target_wz = ROTATION_SIGN * error_x * KP_TURN
        target_wz = clamp(target_wz, -MAX_WZ, MAX_WZ)

    stepped_wz = limit_step(target_wz, previous_wz, MAX_WZ_STEP)
    return WZ_FILTER_ALPHA * stepped_wz + (1.0 - WZ_FILTER_ALPHA) * previous_wz


def draw_debug(frame, tracker, candidates, target, filtered_area, filtered_cx, error_x, vx, vy, wz, wheels):
    frame_height, frame_width = frame.shape[:2]
    center_x = frame_width // 2
    x1, y1, x2, y2 = tracker.roi_bounds(frame_width, frame_height)

    cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 255, 0), 1)
    cv2.line(frame, (center_x, 0), (center_x, frame_height), (255, 255, 0), 1)

    for candidate in candidates:
        x, y, w, h = int(candidate["x"]), int(candidate["y"]), int(candidate["w"]), int(candidate["h"])
        cv2.rectangle(frame, (x, y), (x + w, y + h), (80, 80, 80), 1)

    if target is None:
        cv2.putText(frame, "No locked red board", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)
    else:
        x, y, w, h = int(target["x"]), int(target["y"]), int(target["w"]), int(target["h"])
        cx, cy = int(target["cx"]), int(target["cy"])
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(frame, (cx, cy), 6, (255, 0, 0), -1)
        cv2.circle(frame, (int(filtered_cx), cy), 6, (0, 255, 255), -1)
        cv2.putText(frame, f"Area:{int(target['area'])} filt:{int(filtered_area)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
        cv2.putText(frame, f"ratio:{target['aspect_ratio']:.2f} extent:{target['extent']:.2f} solidity:{target['solidity']:.2f}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
        cv2.putText(frame, f"cx:{cx} filt_cx:{int(filtered_cx)} error:{int(error_x)}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)

    cv2.putText(frame, f"locked:{tracker.locked} lock:{tracker.lock_count}/{tracker.config.lock_required_frames} lost:{tracker.lost_count}/{tracker.config.lost_frames_to_unlock}", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)

    lf, rf, lr, rr = wheels
    cv2.putText(frame, f"vx:{vx:.2f} vy:{vy:.2f} wz:{wz:.2f}", (10, frame_height - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(frame, f"LF:{lf} RF:{rf} LR:{lr} RR:{rr}", (10, frame_height - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--serial-port", default="/dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--camera-index", type=int, default=CAMERA_INDEX)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--no-display", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    motor = MotorSerial(SerialConfig(args.serial_port, args.baudrate, dry_run=args.dry_run))
    tracker = RedBoardTracker(RedTrackingConfig())

    cap = None
    filtered_area = None
    filtered_cx = None
    previous_wz = 0.0
    last_send_time = 0.0

    try:
        motor.open()
        cap = cv2.VideoCapture(args.camera_index)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

        if not cap.isOpened():
            raise RuntimeError("Cannot open camera")

        while True:
            ok, frame = cap.read()
            if not ok:
                motor.stop()
                time.sleep(0.1)
                continue

            frame_height, frame_width = frame.shape[:2]
            mask, candidates, target, locked = tracker.process(frame)

            vx, vy, wz = 0.0, 0.0, 0.0
            error_x = 0.0

            if target is None:
                filtered_area = None
                filtered_cx = None
                previous_wz = 0.0
            else:
                filtered_area = low_pass(filtered_area, target["area"], AREA_FILTER_ALPHA)
                filtered_cx = low_pass(filtered_cx, target["cx"], CX_FILTER_ALPHA)
                error_x = filtered_cx - (frame_width // 2)

                if locked:
                    vy = compute_vy(filtered_area, error_x)
                    wz = compute_wz(error_x, previous_wz)
                    previous_wz = wz
                else:
                    previous_wz = 0.0

            wheels = mecanum_mix(vx, vy, wz, SCALE)

            now = time.time()
            if now - last_send_time >= SEND_INTERVAL:
                motor.send(wheels)
                last_send_time = now

            if not args.no_display:
                draw_debug(
                    frame,
                    tracker,
                    candidates,
                    target,
                    filtered_area or 0.0,
                    filtered_cx or frame_width // 2,
                    error_x,
                    vx,
                    vy,
                    wz,
                    wheels,
                )
                cv2.imshow("Vision Control", frame)
                cv2.imshow("Red Mask", mask)
                if (cv2.waitKey(1) & 0xFF) == ord("q"):
                    break

    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        try:
            motor.stop()
        except Exception:
            pass
        motor.close()
        if cap is not None:
            cap.release()
        if not args.no_display:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

