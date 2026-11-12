# IE415 phone-laptop visual-inertial SLAM

A course project prototype for Android phone capture and laptop-side ArUco mapping and trajectory correction. The official IE415 handout is retained as Project.pdf.

## System overview

1. The Android app uses ARCore for camera frames, camera pose, and image intrinsics.
2. It records raw accelerometer and gyroscope events with their Android timestamps.
3. The app exports a self-contained ZIP through Android's document picker.
4. The laptop detects known ArUco tags in the images, estimates camera pose against a surveyed marker map, and corrects the continuous ARCore VIO trajectory with a six-degree-of-freedom error-state EKF.
5. The laptop saves baseline and corrected trajectories, marker diagnostics, metrics, and a browser-viewable HTML report.

ARCore is the reused visual-inertial baseline. The project code owns the data workflow, marker-map constraints, correction filter, comparisons, and reporting. The markers therefore affect the estimated trajectory; the project does not just draw them over a finished phone path.

## Android app

Requirements:

- Android 7.0 or later.
- An ARCore-supported phone with camera, accelerometer, and gyroscope.
- Google Play Services for AR installed.
- Android SDK platform 34 and JDK 17 or later for building.

Build the debug APK from macOS, Linux, or Windows:

    cd android-app
    ./gradlew assembleDebug

The APK is written to android-app/app/build/outputs/apk/debug/app-debug.apk. Install with Android Studio or:

    adb install -r app/build/outputs/apk/debug/app-debug.apk

Open IE415 SLAM Capture, grant camera access, and wait for ARCore tracking to start. Tap Start, walk through the marked room, tap Stop, then tap Export ZIP and choose a location. The app samples camera images at about 10 Hz and logs IMU readings as they arrive. Keep one rear-camera mode active for a whole experiment.

## Python laptop setup

Python 3.11 or later is recommended.

    cd laptop
    python3 -m venv .venv
    source .venv/bin/activate
    python -m pip install -r requirements.txt

On Windows, activate the environment with:

    .venv\Scripts\activate

Create printable marker images. The sample uses dictionary DICT_4X4_50 and IDs 10 through 17:

    python -m slam_pipeline make-markers --out ../work/markers --ids 10 11 12 13 14 15 16 17 --dictionary DICT_4X4_50

Print the files at the same physical size. Measure the black marker square edge after printing; use that measurement in the map file. Spread about 6 to 10 markers across different walls and heights. Avoid putting all tags in one cluster.

## Map and calibration

Copy marker_map.example.json and edit the marker IDs, physical size, positions, and rotations to match the room. Distances use metres. T_world_marker maps the local printed marker frame into the room frame. The example assumes a wall marker's local +z points out from the print and into the room; markers on a second wall need a corresponding rotation.

For a measured camera distortion model, make a board image and print it at 100% scale:

    python -m slam_pipeline make-board --out ../work/chessboard.png --columns 10 --rows 7

Capture at least 8 sharp images with the same phone lens and image mode used in the experiment. The calibration command expects 9 by 6 inner corners and 24 mm squares by default:

    python -m slam_pipeline calibrate --images ../work/calibration-images --out camera_calibration.json --columns 9 --rows 6 --square-size-m 0.024

The app also stores ARCore CPU-image intrinsics in each frame. If no calibration file is provided, the laptop uses those intrinsics and assumes zero distortion. For final quantitative work, prefer a measured calibration file.

## Process an Android run

Copy the exported ZIP to the laptop, then run:

    python -m slam_pipeline process --session ../work/ie415_session.zip --map marker_map.json --out ../work/results/run_01

Add --calibration camera_calibration.json if available. Open work/results/run_01/index.html for the report. The output includes:

- trajectory_baseline.csv: ARCore VIO after alignment to the first mapped marker.
- trajectory_corrected.csv: VIO trajectory with ArUco pose updates.
- marker_observations.csv: detected IDs, reprojection error, filter acceptance, and correction size.
- summary.json and time_sync.json: run, IMU-rate, and timing diagnostics.
- annotated images and 2D/3D trajectory plots.

The observation CSV also includes each mapped marker's camera-relative 4x4 pose. See [docs/DATA_FORMAT.md](docs/DATA_FORMAT.md) for the schema and [docs/IE415_CONTROL_CONNECTION.md](docs/IE415_CONTROL_CONNECTION.md) for the estimator equations and course connection.

To create an initial persistent map from an exploratory run:

    python -m slam_pipeline build-map --session ../work/exploration.zip --out marker_map.json --dictionary DICT_4X4_50 --marker-length-m 0.16

That map uses the first tracked camera frame as its origin and inherits its pose uncertainty. For final evaluation, replace or validate it using measured marker positions and orientations.

To run the included synthetic end-to-end example:

    python -m slam_pipeline demo --out ../work/demo

This produces a browser report and a synthetic ground-truth comparison. It is only a software demonstration, not experimental evidence.

## Ground truth and experiments

Optionally put ground_truth.csv inside the unzipped session folder with columns timestamp_ns,x,y,z. Coordinates must use the same room frame as marker_map.json and the phone's monotonic timebase. The report calculates ATE RMSE, mean position error, endpoint error, and drift per metre when at least two reference points overlap the run.

Run and save separate datasets for:

1. A straight, L-shaped, or rectangular path.
2. A measurable marker-loss segment.
3. A closed loop that re-observes earlier marker IDs.
4. A repeat run using the saved map.
5. One difficult condition such as oblique views, occlusion, motion blur, or poor lighting.

Keep at least one independent reference route or surveyed point out of the correction map when evaluating accuracy.

## Known scope

- The Android app requires an ARCore-compatible device.
- ARCore's camera-IMU fusion is reused, not reimplemented; the report describes it as a reused estimator. Raw IMU data is logged for synchronization and sensor analysis.
- Camera-to-IMU extrinsics are handled internally by ARCore; the project does not claim to recalibrate them.
- The first version corrects the current pose with an EKF. A full pose graph for retroactively refining old keyframes is a possible extension.
- Phone sessions use a monotonic timestamp for both streams, but each device must be checked. The time-offset estimator needs visible rotation; straight-line data may not excite it enough.
- A map created from one exploratory run is not surveyed ground truth. Measure marker geometry independently for final evaluation.

See docs/IE415_CONTROL_CONNECTION.md for the state-estimation formulation and docs/DATA_FORMAT.md for the session schema.
See docs/PROJECT_PLAN.md for requirement traceability, the five experiment protocol, milestone status, and the distinction between course requirements and supplementary research suggestions.
