# Phone session format

The Android app exports one ZIP containing a session directory:

    ie415_YYYYMMDD_HHMMSS_mmm/
      manifest.json
      frames.csv
      imu.csv
      images/
        frame_000001.jpg
        ...

frames.csv stores the ARCore camera-frame timestamp in nanoseconds, image filename, camera pose quaternion (x,y,z,w), translation in metres, tracking state, and CPU-image intrinsics. imu.csv stores SensorEvent timestamps, sensor label, three-axis values, and Android accuracy. Accelerometer values use m/s^2; gyroscope values use rad/s. Camera and IMU timestamps are recorded from their native Android monotonic clocks.

The laptop keeps the original phone data and writes derived outputs separately:

    trajectory_baseline.csv
    trajectory_corrected.csv
    marker_observations.csv
    summary.json
    trajectory_xy.png
    trajectory_3d.png
    annotated/
    index.html

An optional ground_truth.csv belongs inside the session directory and uses:

    timestamp_ns,x,y,z

The timestamps must use the phone session's monotonic clock. The recommended room frame has x and y on the floor and z upward. If a path was measured at discrete times, use those phone timestamps or document the interpolation method used for comparison.

`marker_observations.csv` has one frame-level row. Its `relative_marker_poses_json` field contains each mapped marker observation as a JSON record with its ID and `T_camera_marker` 4x4 transform. The row also records multi-marker map-pose quality, filter decision, correction magnitude, and reason.
