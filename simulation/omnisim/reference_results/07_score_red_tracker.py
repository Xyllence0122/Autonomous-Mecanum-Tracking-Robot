#!/usr/bin/env python3
# Copyright 2026 OmniLink.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Written for the OmniSim synthetic-data evaluation of
#
# https://github.com/Xyllence0122/Autonomous-Mecanum-Tracking-Robot
#
# It calls that project's RedBoardTracker API but does not modify or include
# any of its code. Redistribution is permitted with attribution; this notice
# must be retained. Nothing here implies that OmniLink endorses the project.
"""Run your project's OWN detector against the
OmniSim export, and score it frame-by-frame against exact segmentation
ground truth.

Your raspberry_pi/red_tracking.py is a pure library:
RedBoardTracker.process(bgr_frame).
It is imported UNMODIFIED -- no thresholds touched, no logic reimplemented.

Pipeline per frame:
1896x1113 RGB --central crop to 4:3 (1484x1113)--> --resize INTER_AREA-->
640x480
(your vision_control.py sets FRAME_WIDTH=640, FRAME_HEIGHT=480)
The instance PNG gets the identical crop + INTER_NEAREST resize, so ground
truth
lives in YOUR pixel coordinates.
"""
import csv
import json
import math
import os
import sys

import cv2
import numpy as np

REPO = sys.argv[1]  # your repo checkout (raspberry_pi/red_tracking.py)
SRC = sys.argv[2]  # the synth raw/ dir
OUT = sys.argv[3]  # output dir
sys.path.insert(0, os.path.join(REPO, "raspberry_pi"))
from red_tracking import RedBoardTracker, RedTrackingConfig  # noqa: E402
# (YOUR code, unmodified)

TARGET_DEF = "RED_TARGET"
TARGET_WORLD_POS = (0.35, 0.0, 0.55)
FW, FH = 640, 480

os.makedirs(OUT, exist_ok=True)
os.makedirs(os.path.join(OUT, "frames_640x480"), exist_ok=True)


def to_camera_frame(img, interp):
    h, w = img.shape[:2]
    cw = int(round(h * FW / FH))  # 4:3 crop width
    x0 = (w - cw) // 2
    return cv2.resize(
        img[:, x0:x0 + cw],
        (FW, FH),
        interpolation=interp
    ), x0, cw


def mask_stats(mask):
    ys, xs = np.nonzero(mask)
    if len(xs) == 0:
        return None

    x, y = int(xs.min()), int(ys.min())
    w, h = int(xs.max() - x + 1), int(ys.max() - y + 1)

    return {
        "px": int(len(xs)),
        "bbox_x": x,
        "bbox_y": y,
        "bbox_w": w,
        "bbox_h": h,
        "bbox_cx": x + w / 2.0,
        "bbox_cy": y + h / 2.0,
        "centroid_x": float(xs.mean()),
        "centroid_y": float(ys.mean()),
    }


samples = sorted(d for d in os.listdir(SRC) if d.startswith("sample_"))
recs = []

for s in samples:
    sd = os.path.join(SRC, s)
    mp = [f for f in os.listdir(sd) if f.startswith("meta_")]

    if not mp:
        continue

    frame = mp[0][5:-5]

    meta = json.load(
        open(
            os.path.join(sd, "meta_%s.json" % frame),
            encoding="utf-8"
        )
    )

    rgb_full = cv2.imread(
        os.path.join(sd, "rgb_%s.png" % frame),
        cv2.IMREAD_COLOR
    )

    inst_full = cv2.imread(
        os.path.join(sd, "inst_%s.png" % frame),
        cv2.IMREAD_COLOR
    )

    dep_full = cv2.imread(
        os.path.join(sd, "depth_%s.png" % frame),
        cv2.IMREAD_UNCHANGED
    )

    ids_full = (
        inst_full[:, :, 2].astype(np.int64)
        + inst_full[:, :, 1].astype(np.int64) * 256
        + inst_full[:, :, 0].astype(np.int64) * 65536
    )

    tid = [
        e["id"]
        for e in meta["instances"]
        if e.get("def") == TARGET_DEF
    ]
    tid = tid[0] if tid else None

    full_mask = (
        ids_full == tid
        if tid is not None
        else np.zeros(ids_full.shape, bool)
    )

    gt_full = mask_stats(full_mask)

    rgb_cam, x0, cw = to_camera_frame(
        rgb_full,
        cv2.INTER_AREA
    )

    ids_cam, _, _ = to_camera_frame(
        ids_full.astype(np.int32),
        cv2.INTER_NEAREST
    )

    cam_mask = (
        ids_cam == tid
        if tid is not None
        else np.zeros(ids_cam.shape, bool)
    )

    gt = mask_stats(cam_mask)

    # true range to the board: exact from camera extrinsics, plus the
    # depth-pass mean
    cp = meta["camera"]["position"]
    rng_exact = math.dist(cp, TARGET_WORLD_POS)

    dep_mm = (
        dep_full[full_mask]
        if gt_full
        else np.array([], np.uint16)
    )

    dep_mm = dep_mm[dep_mm > 0]

    rng_depth = (
        float(dep_mm.mean()) / 1000.0
        if dep_mm.size
        else None
    )

    cv2.imwrite(
        os.path.join(
            OUT,
            "frames_640x480",
            "%s.png" % s
        ),
        rgb_cam
    )

    recs.append({
        "sample": s,
        "frame": frame,
        "rgb": rgb_cam,
        "gt": gt,
        "gt_full": gt_full,
        "cam_pos": cp,
        "range_m": rng_exact,
        "range_depth_m": rng_depth,
        "sun": meta["light"]["sun_direction"],
        "cloud": meta["light"]["cloud_cover"],
        "crop_x0": x0,
        "crop_w": cw,
    })


# camera azimuth about the orbit centre (0,0,0.55) -- used to order the
# pseudo-sweep
for r in recs:
    r["azimuth_deg"] = (
        math.degrees(
            math.atan2(
                r["cam_pos"][1],
                r["cam_pos"][0]
            )
        )
        + 360.0
    ) % 360.0

order = sorted(
    range(len(recs)),
    key=lambda i: recs[i]["azimuth_deg"]
)


# --- pass 1: STATELESS (fresh tracker per frame -- pure detection
# accuracy) -------
for r in recs:
    t = RedBoardTracker(RedTrackingConfig())

    mask, cands, tgt, locked = t.process(
        r["rgb"]
    )

    r["stateless"] = tgt
    r["n_candidates"] = len(cands)
    r["red_mask_px"] = int((mask > 0).sum())


# --- pass 2: STATEFUL, frames fed in ascending azimuth (see the caveat in
# RESULT) --
tracker = RedBoardTracker(RedTrackingConfig())

for i in order:
    r = recs[i]

    _, _, tgt, locked = tracker.process(
        r["rgb"]
    )

    r["stateful"] = tgt
    r["stateful_locked"] = bool(locked)
    r["stateful_lock_count"] = tracker.lock_count


rows = []

for i in order:
    r = recs[i]

    gt, det = r["gt"], r["stateless"]

    row = {
        "sample": r["sample"],
        "azimuth_deg": round(r["azimuth_deg"], 1),
        "range_m": round(r["range_m"], 4),
        "range_from_depth_m": (
            None
            if r["range_depth_m"] is None
            else round(r["range_depth_m"], 4)
        ),
        "cloud_cover": round(r["cloud"], 3),
        "gt_px": gt["px"] if gt else 0,
        "gt_bbox_x": gt["bbox_x"] if gt else None,
        "gt_bbox_y": gt["bbox_y"] if gt else None,
        "gt_bbox_w": gt["bbox_w"] if gt else None,
        "gt_bbox_h": gt["bbox_h"] if gt else None,
        "gt_bbox_cx": round(gt["bbox_cx"], 2) if gt else None,
        "gt_bbox_cy": round(gt["bbox_cy"], 2) if gt else None,
        "gt_centroid_x": round(gt["centroid_x"], 2) if gt else None,
        "gt_centroid_y": round(gt["centroid_y"], 2) if gt else None,
        "gt_aspect": (
            round(gt["bbox_w"] / gt["bbox_h"], 3)
            if gt
            else None
        ),
        "red_mask_px": r["red_mask_px"],
        "n_candidates": r["n_candidates"],
        "detected": det is not None,
        "det_cx": det["cx"] if det else None,
        "det_cy": det["cy"] if det else None,
        "det_area": round(det["area"], 1) if det else None,
        "det_w": det["w"] if det else None,
        "det_h": det["h"] if det else None,
        "det_aspect": round(det["aspect_ratio"], 3) if det else None,
        "det_extent": round(det["extent"], 3) if det else None,
        "det_solidity": round(det["solidity"], 3) if det else None,
        "stateful_detected": r["stateful"] is not None,
        "stateful_locked": r["stateful_locked"],
    }

    if det and gt:
        row["err_cx_px"] = round(
            det["cx"] - gt["bbox_cx"],
            2
        )

        row["err_cy_px"] = round(
            det["cy"] - gt["bbox_cy"],
            2
        )

        row["err_centroid_px"] = round(
            math.hypot(
                det["cx"] - gt["bbox_cx"],
                det["cy"] - gt["bbox_cy"]
            ),
            2
        )

        row["area_ratio_det_over_gt"] = round(
            det["area"] / gt["px"],
            4
        )

        # bbox IoU
        ax1, ay1, ax2, ay2 = (
            det["x"],
            det["y"],
            det["x"] + det["w"],
            det["y"] + det["h"]
        )

        bx1, by1 = (
            gt["bbox_x"],
            gt["bbox_y"]
        )

        bx2, by2 = (
            bx1 + gt["bbox_w"],
            by1 + gt["bbox_h"]
        )

        iw, ih = (
            max(
                0.0,
                min(ax2, bx2) - max(ax1, bx1)
            ),
            max(
                0.0,
                min(ay2, by2) - max(ay1, by1)
            )
        )

        inter = iw * ih

        union = (
            det["w"] * det["h"]
            + gt["bbox_w"] * gt["bbox_h"]
            - inter
        )

        row["bbox_iou"] = (
            round(inter / union, 4)
            if union
            else None
        )

    rows.append(row)


fields = []

for r in rows:
    for k in r:
        if k not in fields:
            fields.append(k)

for r in rows:
    for k in fields:
        r.setdefault(k, None)


with open(
    os.path.join(
        OUT,
        "detector_vs_truth.csv"
    ),
    "w",
    newline="",
    encoding="utf-8"
) as f:
    w = csv.DictWriter(
        f,
        fieldnames=fields
    )
    w.writeheader()
    w.writerows(rows)


with open(
    os.path.join(
        OUT,
        "ground_truth.csv"
    ),
    "w",
    newline="",
    encoding="utf-8"
) as f:
    gtf = [
        "sample",
        "azimuth_deg",
        "range_m",
        "range_from_depth_m",
        "cloud_cover",
        "gt_px",
        "gt_bbox_x",
        "gt_bbox_y",
        "gt_bbox_w",
        "gt_bbox_h",
        "gt_bbox_cx",
        "gt_bbox_cy",
        "gt_centroid_x",
        "gt_centroid_y",
        "gt_aspect",
    ]

    w = csv.DictWriter(
        f,
        fieldnames=gtf,
        extrasaction="ignore"
    )

    w.writeheader()
    w.writerows(rows)


det_rows = [
    r
    for r in rows
    if r["detected"]
]

errs = [
    r["err_centroid_px"]
    for r in det_rows
    if r.get("err_centroid_px") is not None
]

ious = [
    r["bbox_iou"]
    for r in det_rows
    if r.get("bbox_iou") is not None
]

print("frames : %d" % len(rows))

print(
    "target visible (GT>0) : %d"
    % sum(
        1
        for r in rows
        if r["gt_px"] > 0
    )
)

print(
    "your detector fired : %d (stateless, fresh tracker per frame)"
    % len(det_rows)
)

print(
    "your detector LOCKED : %d (stateful, azimuth order)"
    % sum(
        1
        for r in rows
        if r["stateful_locked"]
    )
)

if errs:
    print(
        "centroid error px : mean %.2f median %.2f min %.2f max %.2f"
        % (
            float(np.mean(errs)),
            float(np.median(errs)),
            min(errs),
            max(errs)
        )
    )

    print(
        " (your cx,cy is the bounding-box centre, so it is scored against the "
        "GT bbox centre)"
    )

if ious:
    print(
        "bbox IoU vs truth : mean %.4f min %.4f max %.4f"
        % (
            float(np.mean(ious)),
            min(ious),
            max(ious)
        )
    )

print()

print(
    "%-11s %7s %7s %8s %9s %9s %8s %8s"
    % (
        "sample",
        "az",
        "range",
        "gt_px",
        "det_area",
        "err_px",
        "IoU",
        "locked"
    )
)

for r in rows:
    print(
        "%-11s %7.1f %7.3f %8d %9s %9s %8s %8s"
        % (
            r["sample"],
            r["azimuth_deg"],
            r["range_m"],
            r["gt_px"],
            r["det_area"] if r["detected"] else "-",
            (
                r.get("err_centroid_px")
                if r.get("err_centroid_px") is not None
                else "-"
            ),
            (
                r.get("bbox_iou")
                if r.get("bbox_iou") is not None
                else "-"
            ),
            r["stateful_locked"]
        )
    )