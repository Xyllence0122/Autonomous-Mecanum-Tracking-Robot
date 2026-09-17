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
# https://github.com/Xyllence0122/Autonomous-Mecanum-Tracking-Robot
#
# It calls that project's RedBoardTracker API but does not modify or include
# any of its code. Redistribution is permitted with attribution; this notice
# must be retained. Nothing here implies that OmniLink endorses the project.
"""For every frame, take the largest blob YOUR make_mask() produces and
report which of YOUR
contour_quality() gates rejects it, with the measured value against your
own threshold.

Uses your RedTrackingConfig defaults unmodified; the gate order below is
copied from your
contour_quality() so the 'first failing gate' matches what your code
actually does.
"""
import csv
import json
import math
import os
import sys

import cv2
import numpy as np

REPO, SRC, OUT = sys.argv[1], sys.argv[2], sys.argv[3]
sys.path.insert(0, os.path.join(REPO, "raspberry_pi"))
from red_tracking import RedBoardTracker, RedTrackingConfig  # noqa: E402

cfg = RedTrackingConfig()
FW, FH = 640, 480
TARGET_DEF = "RED_TARGET"
TARGET_WORLD_POS = (0.35, 0.0, 0.55)


def to_cam(img, interp):
    h, w = img.shape[:2]
    cw = int(round(h * FW / FH))
    x0 = (w - cw) // 2
    return cv2.resize(img[:, x0:x0 + cw], (FW, FH), interpolation=interp)


rows = []
for s in sorted(d for d in os.listdir(SRC) if d.startswith("sample_")):
    sd = os.path.join(SRC, s)
    fr = [f for f in os.listdir(sd) if f.startswith("meta_")][0][5:-5]
    meta = json.load(open(os.path.join(sd, "meta_%s.json" % fr),
                          encoding="utf-8"))
    rgb = to_cam(cv2.imread(os.path.join(sd, "rgb_%s.png" % fr),
                            cv2.IMREAD_COLOR), cv2.INTER_AREA)
    inst = cv2.imread(os.path.join(sd, "inst_%s.png" % fr), cv2.IMREAD_COLOR)
    ids = (inst[:, :, 2].astype(np.int64) + inst[:, :, 1].astype(np.int64) * 256
           + inst[:, :, 0].astype(np.int64) * 65536)
    tid = [e["id"] for e in meta["instances"] if e.get("def") == TARGET_DEF][0]
    gtm = to_cam((ids == tid).astype(np.int32), cv2.INTER_NEAREST) > 0
    ys, xs = np.nonzero(gtm)
    gt_w = int(xs.max() - xs.min() + 1) if len(xs) else 0
    gt_h = int(ys.max() - ys.min() + 1) if len(ys) else 0

    cp = meta["camera"]["position"]
    az = (math.degrees(math.atan2(cp[1], cp[0])) + 360.0) % 360.0
    rng = math.dist(cp, TARGET_WORLD_POS)

    t = RedBoardTracker(cfg)
    mask = t.make_mask(rgb)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        rows.append(dict(sample=s, azimuth_deg=round(az, 1), range_m=round(rng, 3),
                         gt_px=int(gtm.sum()), gt_w=gt_w, gt_h=gt_h,
                         gt_aspect=round(gt_w / gt_h, 3) if gt_h else None,
                         blob_px=0, first_failing_gate="no red blob survived make_mask",
                         measured=None, threshold=None, accepted=False))
        continue
    c = max(cnts, key=cv2.contourArea)
    area = float(cv2.contourArea(c))
    x, y, w, h = cv2.boundingRect(c)
    ar = w / float(h) if h else 0.0
    extent = area / float(w * h) if w * h else 0.0
    hull_area = float(cv2.contourArea(cv2.convexHull(c)))
    solidity = area / hull_area if hull_area > 0 else 0.0
    cy = y + h // 2

    gate, measured, thr = None, None, None
    if area < cfg.min_area:
        gate, measured, thr = "min_area", round(area, 1), cfg.min_area
    elif area > cfg.max_area:
        gate, measured, thr = "max_area", round(area, 1), cfg.max_area
    elif ar < cfg.min_aspect_ratio:
        gate, measured, thr = "min_aspect_ratio", round(ar, 3), cfg.min_aspect_ratio
    elif ar > cfg.max_aspect_ratio:
        gate, measured, thr = "max_aspect_ratio", round(ar, 3), cfg.max_aspect_ratio
    elif extent < cfg.min_extent:
        gate, measured, thr = "min_extent", round(extent, 3), cfg.min_extent
    elif solidity < cfg.min_solidity:
        gate, measured, thr = "min_solidity", round(solidity, 3), cfg.min_solidity
    elif cy > int(FH * cfg.max_center_y_ratio):
        gate, measured, thr = "max_center_y_ratio", cy, int(FH *
                                                             cfg.max_center_y_ratio)

    rows.append(dict(sample=s, azimuth_deg=round(az, 1), range_m=round(rng, 3),
                     gt_px=int(gtm.sum()), gt_w=gt_w, gt_h=gt_h,
                     gt_aspect=round(gt_w / gt_h, 3) if gt_h else None,
                     blob_px=int(area), blob_w=w, blob_h=h, blob_aspect=round(ar, 3),
                     blob_extent=round(extent, 3), blob_solidity=round(solidity, 3),
                     first_failing_gate=gate or "(accepted)", measured=measured, threshold=thr,
                     accepted=gate is None))

rows.sort(key=lambda r: r["azimuth_deg"])
fields = []
for r in rows:
    for k in r:
        if k not in fields:
            fields.append(k)
for r in rows:
    for k in fields:
        r.setdefault(k, None)
with open(os.path.join(OUT, "gate_diagnosis.csv"), "w", newline="",
          encoding="utf-8") as f:
    w_ = csv.DictWriter(f, fieldnames=fields)
    w_.writeheader()
    w_.writerows(rows)

print("%-11s %7s %7s %8s %10s %10s %-22s %10s %10s" %
      ("sample", "az", "range", "gt_px", "gt_aspect", "blob_asp",
       "first_failing_gate", "measured", "threshold"))
for r in rows:
    print("%-11s %7.1f %7.3f %8d %10s %10s %-22s %10s %10s" %
          (r["sample"], r["azimuth_deg"], r["range_m"], r["gt_px"], r["gt_aspect"],
           r.get("blob_aspect"), r["first_failing_gate"],
           r["measured"] if r["measured"] is not None else "-",
           r["threshold"] if r["threshold"] is not None else "-"))
acc = sum(1 for r in rows if r["accepted"])
print("\naccepted %d / %d" % (acc, len(rows)))
from collections import Counter
print("rejections by gate:", dict(Counter(r["first_failing_gate"] for r in
                                          rows if not r["accepted"])))