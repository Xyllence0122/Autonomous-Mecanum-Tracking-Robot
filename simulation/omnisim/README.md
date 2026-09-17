# OmniSim Synthetic Evaluation

This directory contains the synthetic perception evaluation performed by the
OmniLink Team using OmniSim for the
[Autonomous-Mecanum-Tracking-Robot](https://github.com/Xyllence0122/Autonomous-Mecanum-Tracking-Robot)
project.

The original `raspberry_pi/red_tracking.py` implementation was imported and
evaluated without modification. No detector thresholds or tracking logic were
reimplemented or tuned for the evaluation.

## Evaluation Pipeline

The evaluation uses OmniSim to generate synchronized:

- RGB images
- depth maps
- per-instance segmentation
- camera and scene metadata

The rendered RGB frames are centrally cropped to 4:3 and resized to 640×480,
matching the input resolution expected by the original perception pipeline.

The project's own `RedBoardTracker.process()` implementation is then evaluated
against exact instance-segmentation ground truth.

## Baseline Results

The evaluation contains 16 synthetic viewpoints.

| Metric | Result |
|---|---:|
| Detection rate | 8 / 16 |
| Median centroid error | 1.93 px |
| Mean centroid error | 3.54 px |
| Mean bounding-box IoU | 0.9257 |
| False positives | 0 |

### Miss Diagnosis

All eight missed detections passed the color segmentation stage.

The failures were caused by subsequent geometric gates:

| Gate | Rejections |
|---|---:|
| `min_aspect_ratio` | 6 |
| `min_area` | 2 |

The evaluation therefore identified the aspect-ratio gate as the main synthetic
failure mode under off-axis views of the target board.

## Files

### Reference Results

- `reference_results/01_ground_truth.csv`  
  Per-frame exact ground-truth information.

- `reference_results/02_detector_vs_truth.csv`  
  Detector output compared with synthetic ground truth.

- `reference_results/03_gate_diagnosis.csv`  
  Per-frame diagnosis of the first detector gate responsible for rejection.

### Simulation Setup

- `scene/04_red_target_track.omniworld`  
  OmniSim scene used for the evaluation.

- `config/05a_run_configuration.txt`  
  Environment, dependency, randomization, camera, and deterministic-run
  configuration.

- `config/05b_scene_and_scripts.txt`  
  Scene details, target/distractor configuration, export configuration, and
  regeneration procedure.

### Evaluation Tools

- `scripts/06_diagnose_gates.py`  
  Gate-level instrumentation for the original red-target detector.

- `scripts/07_score_red_tracker.py`  
  Evaluation harness that directly imports and runs the project's original
  `RedBoardTracker`.

## Reproduction

The supplied configuration includes the information necessary to regenerate the
synthetic evaluation using OmniSim.

Reference configuration:

- Random seed: `21`
- Samples: `16`

See `config/05a_run_configuration.txt` and
`config/05b_scene_and_scripts.txt` for the complete regeneration procedure.

OmniSim:

https://github.com/omnilink-tech/omnisim

The rendered image dataset itself is not stored in this repository. The
configuration, scene, evaluation scripts, and reference results are included so
that the dataset can be regenerated if required.

## Limitations

This evaluation is a synthetic perception benchmark, not a full robot
simulation.

The evaluation does not establish real-world transfer performance. In
particular, it does not model:

- photorealistic appearance
- sensor noise
- motion blur
- rolling shutter
- robot physics
- a temporally continuous robot trajectory

The 16 samples are independent synthetic viewpoints.

The reported zero false positives were measured against the distractors present
in this particular synthetic scene and should not be interpreted as a general
false-positive guarantee.

## Attribution and License

The OmniSim simulation environment and evaluation tooling in this directory were
provided by the OmniLink Team.

The provided OmniLink files are distributed under the Apache License 2.0.
The original license notices in the supplied scripts and configuration files
must be retained.

OmniSim and OmniLink attribution does not imply endorsement of this project.

The original Autonomous-Mecanum-Tracking-Robot source code remains under its
own licensing terms.