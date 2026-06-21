#!/usr/bin/env python3
"""Red board detection, contour filtering, ROI filtering, and target lock."""

from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np


Target = Dict[str, float]


@dataclass
class RedTrackingConfig:
    lower_red1: Tuple[int, int, int] = (0, 150, 70)
    upper_red1: Tuple[int, int, int] = (10, 255, 255)
    lower_red2: Tuple[int, int, int] = (170, 150, 70)
    upper_red2: Tuple[int, int, int] = (180, 255, 255)

    roi_x_min_ratio: float = 0.05
    roi_x_max_ratio: float = 0.95
    roi_y_min_ratio: float = 0.00
    roi_y_max_ratio: float = 0.90

    min_area: float = 2500.0
    max_area: float = 160000.0
    min_aspect_ratio: float = 0.8
    max_aspect_ratio: float = 3.5
    min_extent: float = 0.35
    min_solidity: float = 0.70
    max_center_y_ratio: float = 0.82

    lock_required_frames: int = 5
    max_target_jump_px: float = 140.0
    lost_frames_to_unlock: int = 12


class RedBoardTracker:
    def __init__(self, config: RedTrackingConfig):
        self.config = config
        self.locked = False
        self.lock_count = 0
        self.lost_count = 0
        self.last_target: Optional[Target] = None

    def roi_bounds(self, frame_width: int, frame_height: int) -> Tuple[int, int, int, int]:
        cfg = self.config
        x1 = int(frame_width * cfg.roi_x_min_ratio)
        x2 = int(frame_width * cfg.roi_x_max_ratio)
        y1 = int(frame_height * cfg.roi_y_min_ratio)
        y2 = int(frame_height * cfg.roi_y_max_ratio)
        return x1, y1, x2, y2

    def make_mask(self, frame: np.ndarray) -> np.ndarray:
        cfg = self.config
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask1 = cv2.inRange(hsv, np.array(cfg.lower_red1), np.array(cfg.upper_red1))
        mask2 = cv2.inRange(hsv, np.array(cfg.lower_red2), np.array(cfg.upper_red2))
        mask = cv2.bitwise_or(mask1, mask2)

        frame_height, frame_width = frame.shape[:2]
        x1, y1, x2, y2 = self.roi_bounds(frame_width, frame_height)
        roi_mask = np.zeros_like(mask)
        roi_mask[y1:y2, x1:x2] = 255
        mask = cv2.bitwise_and(mask, roi_mask)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        return mask

    def contour_quality(
        self,
        contour: np.ndarray,
        frame_width: int,
        frame_height: int,
    ) -> Optional[Target]:
        cfg = self.config
        area = float(cv2.contourArea(contour))
        if area < cfg.min_area or area > cfg.max_area:
            return None

        x, y, w, h = cv2.boundingRect(contour)
        if w <= 0 or h <= 0:
            return None

        aspect_ratio = w / float(h)
        if aspect_ratio < cfg.min_aspect_ratio or aspect_ratio > cfg.max_aspect_ratio:
            return None

        extent = area / float(w * h)
        if extent < cfg.min_extent:
            return None

        hull = cv2.convexHull(contour)
        hull_area = float(cv2.contourArea(hull))
        if hull_area <= 0:
            return None

        solidity = area / hull_area
        if solidity < cfg.min_solidity:
            return None

        cx = x + w // 2
        cy = y + h // 2
        if cy > int(frame_height * cfg.max_center_y_ratio):
            return None

        score = area * extent * solidity
        return {
            "area": area,
            "x": float(x),
            "y": float(y),
            "w": float(w),
            "h": float(h),
            "cx": float(cx),
            "cy": float(cy),
            "aspect_ratio": aspect_ratio,
            "extent": extent,
            "solidity": solidity,
            "score": score,
        }

    def find_candidates(self, mask: np.ndarray, frame_width: int, frame_height: int) -> List[Target]:
        contours_info = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = contours_info[0] if len(contours_info) == 2 else contours_info[1]

        candidates: List[Target] = []
        for contour in contours:
            quality = self.contour_quality(contour, frame_width, frame_height)
            if quality is not None:
                candidates.append(quality)

        candidates.sort(key=lambda item: item["score"], reverse=True)
        return candidates

    def choose_target(self, candidates: List[Target]) -> Optional[Target]:
        cfg = self.config
        if not candidates:
            return None

        if not self.locked or self.last_target is None:
            return candidates[0]

        best = None
        best_dist = None
        for candidate in candidates:
            dx = candidate["cx"] - self.last_target["cx"]
            dy = candidate["cy"] - self.last_target["cy"]
            dist = (dx * dx + dy * dy) ** 0.5
            if best is None or dist < best_dist:
                best = candidate
                best_dist = dist

        if best_dist is not None and best_dist <= cfg.max_target_jump_px:
            return best

        return None

    def update_lock(self, target: Optional[Target]) -> bool:
        cfg = self.config
        if target is None:
            self.lost_count += 1
            self.lock_count = 0
            if self.lost_count >= cfg.lost_frames_to_unlock:
                self.locked = False
                self.last_target = None
            return self.locked

        self.lost_count = 0
        self.lock_count = min(self.lock_count + 1, cfg.lock_required_frames)
        if self.lock_count >= cfg.lock_required_frames:
            self.locked = True
        self.last_target = target
        return self.locked

    def process(self, frame: np.ndarray) -> Tuple[np.ndarray, List[Target], Optional[Target], bool]:
        frame_height, frame_width = frame.shape[:2]
        mask = self.make_mask(frame)
        candidates = self.find_candidates(mask, frame_width, frame_height)
        target = self.choose_target(candidates)
        locked = self.update_lock(target)
        return mask, candidates, target, locked

