# IE415 Visual–Inertial SLAM with Fiducial Landmark Anchors

> Working template. Replace every bracketed instruction with measured project information. Do not report synthetic-demo values as physical results.

## Abstract

[Problem, Android–laptop system, method, experiment count, measured result, and main limitation.]

## 1. Problem statement and objectives

[Indoor localization and mapping task; why fixed ArUco markers are used as anchors; project objectives.]

## 2. IE415 concepts

[State estimation, coordinate transforms, observability, sensor uncertainty, calibration, synchronization, and why this project has no motion controller: the person carries the phone.]

## 3. System architecture

[Phone capture, ZIP transfer, laptop processing, dependencies, data flow, and reused versus project-written components. Identify ARCore as the reused VIO estimator.]

## 4. Frames and measurement model

Define `T_A_B` as the transform that maps coordinates from frame B into frame A. State the world, ARCore, OpenCV camera, IMU, and marker axes, units, and quaternion order. Include:

```text
T_world_camera = T_world_marker · inverse(T_camera_marker)
```

[Describe camera intrinsics/distortion, marker corner order, pose covariance, and error sources.]

## 5. Sensing, calibration, and synchronization

[Android device/model, camera mode, image rate, accelerometer/gyroscope rates, timestamp clock, chessboard dimensions, number of calibration images, calibration RMS, and time-offset method/result. State which camera–IMU calibration is supplied internally by ARCore.]

## 6. Mapping and estimator

[Marker dictionary/IDs/size; surveyed room layout and uncertainties; initial map procedure; ARCore relative-pose propagation; six-degree-of-freedom EKF update; covariance assumptions; outlier gate; tracking-loss relocalization.]

## 7. Experimental setup

[Room dimensions, marker-placement table, phone, route measurement method, operator, lighting, and processing computer. Attach the filled experiment log.]

## 8. Experiments

| Run | Scenario | Duration | Frames | Marker visibility | Ground-truth method |
|---|---|---:|---:|---|---|
| E1 | Simple path | [ ] | [ ] | [ ] | [ ] |
| E2 | Marker loss | [ ] | [ ] | [ ] | [ ] |
| E3 | Closed loop | [ ] | [ ] | [ ] | [ ] |
| E4 | Repeat run | [ ] | [ ] | [ ] | [ ] |
| E5 | Difficult condition | [ ] | [ ] | [ ] | [ ] |

## 9. Evaluation metrics

[Define the alignment used and report ATE RMSE, mean position error, endpoint error, drift per metre, marker observation rate, rejected updates, relocalization time when applicable, and processing rate. Describe ground-truth independence and interpolation.]

## 10. Results

| Scenario | Estimator | ATE RMSE (m) | Endpoint error (m) | Drift per metre | Notes |
|---|---|---:|---:|---:|---|
| [ ] | ARCore baseline | [ ] | [ ] | [ ] | [ ] |
| [ ] | ARCore + marker EKF | [ ] | [ ] | [ ] | [ ] |

[Insert report plots and explain when marker updates changed the trajectory. Include only real recordings in this table.]

## 11. Failure analysis and limitations

[Discuss marker loss, motion blur, oblique views, map errors, calibration, synchronization, ARCore tracking failures, current-pose EKF limitations, and the limits of the chosen ground truth.]

## 12. Conclusions and future work

[Evidence-based conclusion, limitations, and prioritized next steps such as factor-graph optimization or direct IMU propagation.]

## References

[Course handout, ARCore, OpenCV ArUco, and any technical sources actually used.]

## Reproducibility appendix

[Repository commit, build commands, Python environment, phone export, map, calibration, ground truth, experiment log, and report output locations.]
