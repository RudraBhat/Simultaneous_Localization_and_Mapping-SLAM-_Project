# IE415 control and estimation connection

## System boundary

The physical platform is a phone carried by a person. The phone estimates its camera motion; the laptop adds explicit room-marker constraints and presents the result. The person supplies the motion, so this project does not claim to implement a motion controller. Its IE415 contribution is autonomous-system state estimation, sensor modeling, observability, coordinate transformations, calibration, and quantitative validation.

## State and measurements

The handout's visual-inertial state is

    x_k = [p_k, v_k, q_k, b_a,k, b_g,k]

where p is position, v is velocity, q is orientation, and b_a and b_g are accelerometer and gyroscope biases. ARCore supplies a continuous camera pose and its internally fused visual-inertial estimate. The phone logger also stores raw accelerometer and gyroscope readings so timing, axes, and sensor quality can be inspected independently.

The laptop estimator maintains a camera pose T_world_camera and its 6x6 pose covariance. Between frames it propagates using the relative ARCore pose. A mapped marker gives an absolute camera-pose observation from the image corners and surveyed marker coordinates. The marker observation is an EKF measurement update, not a graphic overlay.

## Coordinate convention

The code uses T_A_B to mean the transform that maps coordinates in frame B into frame A. The important frames are:

- W: surveyed room frame, with x and y on the floor and z upward.
- C: OpenCV camera frame, +x image-right, +y image-down, +z forward.
- A: ARCore world frame.
- M_i: local frame of marker i; +x follows the printed right edge, +y follows the printed top edge, +z points out of the printed face and into the room.

For a known marker pose and an observed camera-to-marker pose:

    T_W_C = T_W_M * inverse(T_C_M)

ARCore's camera axes differ from OpenCV's, so the importer applies the fixed camera-basis rotation diag(1,-1,-1). A marker map is the world-frame anchor across sessions; an ARCore world origin is not assumed to be persistent. The laptop's x-y plot is a top-down view when the surveyed map follows this room convention.

## Estimator update

The motion-only estimate is the baseline. Given consecutive ARCore camera poses, the laptop computes a relative motion increment and propagates the current corrected estimate. At a mapped-marker observation, it forms a six-dimensional residual:

    r = [p_marker - p_est, Log(R_est^T R_marker)]

The filter weights that residual using measurement covariance derived from tag range, reprojection error, and the number of visible tags. A Mahalanobis gate rejects inconsistent observations. The corrected estimate and the baseline are both saved, so the effect of the fiducials can be evaluated directly.

This is intentionally a small error-state pose filter. A full factor graph is not required for the first working version; it can be added if experiments show that past keyframes need retroactive correction.

## Calibration and timing

The Android logger saves ARCore image intrinsics, camera frame timestamps, and SensorEvent timestamps. OpenCV's chessboard calibration command can estimate a distortion model for the exact camera lens and image mode. The platform's camera-IMU calibration is reused through ARCore and must be identified as such in the report rather than described as a calibration implemented by the project.

Sensor timestamps are retained in nanoseconds. The laptop's import report computes sample rates and can compare the magnitude of camera angular velocity with gyro magnitude to estimate a relative time offset when the recorded motion contains enough rotation. If motion is nearly straight, the offset estimate is marked unavailable rather than guessed.

## Evaluation design

Every run can be processed twice:

1. ARCore motion only.
2. ARCore motion corrected by known ArUco map observations.

The report computes ATE RMSE, mean position error, endpoint error, and drift per metre when an independent ground-truth CSV is supplied. It also reports tracking coverage, mapped-marker observation rate, and estimation processing rate. The experiment set includes a simple walk, marker-loss period, closed loop, repeat run using the stored map, and one difficult condition. For a fair claim, retain at least one surveyed reference point or route measurement that is not used as a correction input.

## Scope and attribution

ARCore is a reused visual-inertial estimator and supplies continuous motion when tags are absent. Project-owned code handles dataset management, marker detection, pose recovery from surveyed marker geometry, pose filtering, baseline comparison, metrics, and visualization. This makes the marker effect inspectable while avoiding a large VIO implementation in C++.
