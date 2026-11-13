# Phone run checklist

Use this before collecting a real dataset. Keep the same camera lens, resolution, marker dictionary, and printed marker size throughout a study.

## Prepare the room

- [ ] Confirm the Android phone is supported by ARCore and has Google Play Services for AR installed.
- [ ] Generate tags from one dictionary and IDs 10–17; print them without resizing.
- [ ] Measure each printed black square. Enter the measured edge length, not the paper size.
- [ ] Place 6–10 tags across more than one wall and height. Record their centers and orientations in a room frame with x/y on the floor and z upward.
- [ ] Replace the illustrative coordinates in `laptop/marker_map.example.json` with the measured room coordinates. The example describes a hypothetical room, not your room.
- [ ] Record each tag in `docs/marker_survey_template.csv`; quaternions use `[x,y,z,w]` and map the printed marker frame into the room frame.
- [ ] Choose an independent ground-truth method before recording: measured floor path, surveyed points, or another documented reference.
- [ ] If doing intrinsic calibration, print the chessboard at 100% and capture at least eight sharp, varied views with the same lens and image mode.

## Capture on Android

- [ ] Open IE415 SLAM Capture and grant camera permission.
- [ ] Wait until ARCore tracking reports `TRACKING`.
- [ ] Tap **Start**, follow the planned path smoothly, and keep the phone in the same rear-camera mode.
- [ ] Include any planned marker-free interval and return to previously seen tags for a closed loop.
- [ ] Tap **Stop**, then **Export ZIP**. Keep the original ZIP unchanged and record its run ID in `experiment_log_template.csv`.

## Check and process on the laptop

- [ ] Activate the Python environment and run `python -m slam_pipeline inspect --session <run.zip>`.
- [ ] Review camera frame count, tracking fraction, IMU counts/rates, timestamp order, and warnings.
- [ ] Process the run with the surveyed map; add the camera calibration file if one was measured.
- [ ] Review annotated frames, marker IDs, reprojection errors, filter rejections, time-sync result, and both trajectory plots.
- [ ] Save the ZIP, map, calibration, ground-truth file, experiment notes, and output report together.
- [ ] Repeat for the simple path, marker-loss path, closed loop, repeat run, and selected difficult condition.

Do not describe synthetic-demo metrics as physical accuracy. A time-offset estimate is only diagnostic; validate its sign and size against real capture behavior before relying on it.
