#!/usr/bin/env python3
"""Interactive HSV tuner for red color segmentation."""

import argparse

import cv2
import numpy as np


def nothing(_):
    pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera-index", type=int, default=0)
    args = parser.parse_args()

    cap = cv2.VideoCapture(args.camera_index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    if not cap.isOpened():
        raise RuntimeError("Cannot open camera")

    cv2.namedWindow("HSV Tuner")
    cv2.createTrackbar("H1 low", "HSV Tuner", 0, 180, nothing)
    cv2.createTrackbar("H1 high", "HSV Tuner", 10, 180, nothing)
    cv2.createTrackbar("H2 low", "HSV Tuner", 170, 180, nothing)
    cv2.createTrackbar("H2 high", "HSV Tuner", 180, 180, nothing)
    cv2.createTrackbar("S low", "HSV Tuner", 150, 255, nothing)
    cv2.createTrackbar("V low", "HSV Tuner", 70, 255, nothing)

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                continue

            h1_low = cv2.getTrackbarPos("H1 low", "HSV Tuner")
            h1_high = cv2.getTrackbarPos("H1 high", "HSV Tuner")
            h2_low = cv2.getTrackbarPos("H2 low", "HSV Tuner")
            h2_high = cv2.getTrackbarPos("H2 high", "HSV Tuner")
            s_low = cv2.getTrackbarPos("S low", "HSV Tuner")
            v_low = cv2.getTrackbarPos("V low", "HSV Tuner")

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask1 = cv2.inRange(hsv, np.array([h1_low, s_low, v_low]), np.array([h1_high, 255, 255]))
            mask2 = cv2.inRange(hsv, np.array([h2_low, s_low, v_low]), np.array([h2_high, 255, 255]))
            mask = cv2.bitwise_or(mask1, mask2)
            result = cv2.bitwise_and(frame, frame, mask=mask)

            cv2.imshow("Camera", frame)
            cv2.imshow("Mask", mask)
            cv2.imshow("Result", result)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("p"):
                print(f"LOWER_RED1 = ({h1_low}, {s_low}, {v_low})")
                print(f"UPPER_RED1 = ({h1_high}, 255, 255)")
                print(f"LOWER_RED2 = ({h2_low}, {s_low}, {v_low})")
                print(f"UPPER_RED2 = ({h2_high}, 255, 255)")

    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

