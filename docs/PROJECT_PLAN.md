# IE415 Phone–Laptop SLAM Project Plan

## Project decisions and source authority

- **User-selected system:** Android phone for sensing and interaction; laptop for replayable estimation, mapping, evaluation, and visualization. Python is the main implementation language.
- **Course requirements:** the eight-page IE415 project handout is the authoritative specification. Its minimum functions, experiments, deliverables, and rubric are requirements for the submission.
- **Supplementary guidance:** the attached 26-page email research report interprets the handout and suggests design options, experiments, metrics, and extensions. It is useful guidance, but its extra recommendations are not additional instructor requirements.
- **Implementation choice:** ARCore supplies the phone's continuous visual–inertial camera pose. Our Python code detects mapped ArUco landmarks and feeds their pose constraints into a six-degree-of-freedom EKF. This keeps the system understandable and makes the marker correction measurable without pretending to reimplement a complete production VIO stack.

## Architecture

```text
Android phone
  rear camera + ARCore camera pose + camera intrinsics
  accelerometer + gyroscope + native timestamps
  start / stop / status / ZIP export
           │  file transfer
           ▼
Laptop Python pipeline
  dataset import → calibration/synchronization diagnostics
  → ArUco ID/corner detection → camera-to-marker pose
  → persistent marker-map pose → VIO propagation + EKF updates
  → baseline comparison → metrics, plots, annotated frames, HTML report
```

Capture and estimation are separate so the same phone session can be replayed after changes to the map, calibration, or estimator. Processing is offline; real-time operation is not required by the handout.

## Coordinate frames and estimation

- `W`: room/world frame. Use x and y on the floor, z upward; units are metres.
- `C`: OpenCV camera frame. +x points to image right, +y down, and +z forward.
- `M_i`: marker frame. +x follows the printed right edge, +y follows the printed top edge, and +z points out from the printed face.
- `A`: ARCore session frame. The session origin may change between runs, so a mapped marker anchors each run into W.

For each image, OpenCV estimates a direct `T_camera_marker` from the tag corners. With a surveyed marker pose `T_world_marker`, it yields a world camera-pose observation:

```text
T_world_camera = T_world_marker · inverse(T_camera_marker)
```

ARCore relative camera motion propagates the current estimate between images. The marker pose is an EKF measurement update. Large inconsistent observations are rejected; after an ARCore tracking gap, a valid mapped marker can initialize the state again. When no mapped tag is visible, the trajectory continues from ARCore motion.

The estimator state here is the camera pose and a 6×6 pose covariance. ARCore internally estimates visual–inertial motion using its own richer state, including inertial quantities. We log raw accelerometer and gyroscope data for timing, rates, and sensor-quality analysis, but do not claim that this project independently estimates IMU biases or camera–IMU extrinsics.

## Requirement traceability

| IE415 minimum function | Project implementation | Evidence / remaining field work |
|---|---|---|
| Moving camera sequence | Android app records ARCore CPU camera images at about 10 Hz | `frames.csv`, saved JPEG frames; capture on a supported Android phone |
| IMU measurements | Android accelerometer and gyroscope events | `imu.csv`; check sample counts and rates for each run |
| Timestamping and synchronization | Nanosecond timestamps are stored for both streams; laptop estimates a gyro/camera rotation offset when motion excites it | `time_sync.json`; verify on real recordings, especially with turns |
| Camera calibration | Laptop chessboard calibration tool; ARCore image intrinsics are the fallback | Save the calibration output for the same camera mode and image size |
| Marker detection and unique IDs | OpenCV ArUco dictionary detection | Annotated images and detected IDs in `marker_observations.csv` |
| Marker pose relative to camera | Per-tag `T_camera_marker` from image corners and camera intrinsics | `relative_marker_poses_json` in `marker_observations.csv` |
| Continuous trajectory | ARCore camera motion, including intervals without mapped tags | Baseline and corrected trajectory CSVs |
| Construct or load a marker map | JSON surveyed map plus exploratory-run map builder | `marker_map.json`; measure real marker positions and orientations |
| Trajectory relative to map | Surveyed marker poses anchor the ARCore session | XY/3D plots and browser report |
| Repeated marker constraint | Six-DoF pose EKF update; mapped-marker relocalization after tracking loss | Accepted/rejected updates and correction magnitudes in observation CSV |
| Baseline comparison | Aligned ARCore trajectory versus marker-corrected trajectory | Compare with independent ground truth; synthetic demo only checks software behavior |
| Quantitative evaluation | ATE RMSE, mean position error, endpoint error, and drift per metre | Requires real ground-truth measurements before reporting physical accuracy |

## Experimental protocol

Place about 6–10 printed tags across multiple walls and heights. Record each tag's ID, measured black-square edge length, position, orientation, and measurement uncertainty. Keep a small measured route or reference point independent from the map for evaluation.

1. **Simple path:** straight, L-shaped, or rectangular walk with stable lighting.
2. **Marker loss:** include a known segment where no map tag is visible; compare drift during the gap.
3. **Closed loop:** return to the start area and re-observe earlier IDs; inspect the EKF update and endpoint error.
4. **Repeat run:** start a new phone session and reuse the saved marker map; measure alignment and recovery time.
5. **Difficult condition:** choose and document blur, oblique angle, partial occlusion, dim lighting, or sparse visibility.

For each run, preserve the raw phone ZIP, map, calibration, ground-truth method, and generated report together. Compare baseline and corrected trajectories on the same timestamps. Report physical results only for runs with independent ground truth; the included synthetic sequence is a software smoke demonstration, not evidence of room accuracy.

## Build milestones and submission package

| Milestone | Work | Current status |
|---|---|---|
| M1 | Problem framing, hybrid architecture, frames, software choices | Implemented and documented |
| M2 | Android camera/IMU acquisition, timestamped dataset, camera calibration | App and calibration workflow implemented; physical phone run still required |
| M3 | ArUco detection and camera-relative pose | Implemented in the Python pipeline |
| M4 | Continuous VIO baseline | ARCore pose stream integrated; its estimator is reused and identified as such |
| M5 | Persistent map, repeated IDs, map-relative visualization | Map load/build, EKF correction, relocalization, and report implemented |
| M6 | Drift correction and baseline comparison | Implemented; synthetic demo available; real comparative evidence pending |
| M7 | Required room experiments, final analysis, report, presentation, demonstration video | Requires the team's physical data and final course deliverables |

The repository includes source and build instructions, a representative synthetic dataset generator, example marker map and ground truth, course-method documentation, and a report generator. Before final submission, add at least one real representative dataset and metadata, measured map/calibration files, the five experiment results, technical report, presentation, and demonstration video.

## Scope and limitations

- The Android app is a sensor and dataset capture tool. The laptop runs the estimator and generates the results page.
- The EKF corrects the current pose; it does not retroactively optimize every past keyframe as a full factor graph would.
- ARCore's VIO, camera–IMU calibration, and sensor fusion are reused. Project-owned contributions include data capture/export, map handling, marker pose measurements, robust pose filtering, evaluation, and reporting.
- A map built from one exploratory session has an arbitrary origin and inherits that session's uncertainty. Use surveyed marker poses for a defensible room frame and repeated-run evaluation.
- The repository's synthetic run verifies pipeline execution only. Real camera behavior, Android sensor rates, APK installation, room accuracy, and physical marker placement still need validation on an ARCore-compatible Android phone.
