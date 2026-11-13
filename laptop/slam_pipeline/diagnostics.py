from __future__ import annotations

import csv
from collections import Counter
from pathlib import Path

import cv2

from .session import read_frames, read_imu_summary
from .time_sync import estimate_time_offset


def inspect_session(session_dir: Path) -> dict[str, object]:
    """Summarize capture completeness and flag common problems before processing."""
    session_dir = session_dir.resolve()
    frames_path = session_dir / "frames.csv"
    if not frames_path.exists():
        raise FileNotFoundError(f"No frames.csv found in {session_dir}")

    with frames_path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError("frames.csv contains no camera frames.")

    timestamps = [int(row["timestamp_ns"]) for row in rows]
    tracking_counts = Counter((row.get("tracking_state") or "UNKNOWN").upper() for row in rows)
    missing_images = [row.get("image_file", "") for row in rows if not (session_dir / row["image_file"]).is_file()]
    resolutions = sorted(
        {
            f"{row.get('width', '?')}x{row.get('height', '?')}"
            for row in rows
        }
    )
    frames = read_frames(session_dir)
    unreadable_images = [frame.image_path for frame in frames if cv2.imread(str(frame.image_path)) is None]
    imu = read_imu_summary(session_dir)
    duration_s = max(0.0, (timestamps[-1] - timestamps[0]) / 1e9)
    camera_rate_hz = (len(timestamps) - 1) / duration_s if duration_s > 0 else 0.0
    tracking_rate = tracking_counts.get("TRACKING", 0) / len(rows)
    timestamp_monotonic = all(current > previous for previous, current in zip(timestamps, timestamps[1:]))
    time_sync = (
        estimate_time_offset(session_dir, frames)
        if timestamp_monotonic
        else {"status": "unavailable", "reason": "camera timestamps are not strictly increasing"}
    )

    warnings: list[str] = []
    notes: list[str] = []
    if missing_images:
        warnings.append(f"{len(missing_images)} image file(s) listed in frames.csv are missing.")
    if not timestamp_monotonic:
        warnings.append("Camera timestamps are not strictly increasing.")
    if len(frames) != len(rows):
        warnings.append(f"Only {len(frames)} of {len(rows)} frame images could be read.")
    if unreadable_images:
        warnings.append(f"{len(unreadable_images)} image file(s) exist but cannot be decoded.")
    if tracking_rate < 0.8:
        warnings.append(f"ARCore tracking was active for only {tracking_rate:.0%} of logged frames.")
    if len(resolutions) > 1:
        warnings.append("Image dimensions change during the session; use one camera mode per run.")
    if int(imu.get("accelerometer_samples", 0)) == 0:
        warnings.append("No accelerometer samples were found in imu.csv.")
    if int(imu.get("gyroscope_samples", 0)) == 0:
        warnings.append("No gyroscope samples were found in imu.csv.")
    if camera_rate_hz < 5.0:
        warnings.append(f"Camera sampling rate is low ({camera_rate_hz:.1f} Hz).")
    if imu.get("duration_s") and abs(float(imu["duration_s"]) - duration_s) > 1.0:
        warnings.append("Camera and IMU session durations differ by more than one second.")
    if time_sync.get("status") == "insufficient_rotation":
        notes.append("The run has too little turning motion to estimate a camera-to-IMU time offset; use a run with turns to validate that diagnostic.")
    elif time_sync.get("status") == "unavailable":
        notes.append("The camera-to-IMU offset diagnostic is unavailable: " + str(time_sync.get("reason", "unknown reason")))

    return {
        "session": session_dir.name,
        "camera_frames_in_csv": len(rows),
        "camera_images_readable": len(frames) - len(unreadable_images),
        "undecodable_image_count": len(unreadable_images),
        "missing_image_count": len(missing_images),
        "duration_s": round(duration_s, 3),
        "camera_rate_hz": round(camera_rate_hz, 2),
        "camera_timestamps_strictly_increasing": timestamp_monotonic,
        "image_resolutions": resolutions,
        "tracking_state_counts": dict(tracking_counts),
        "arcore_tracking_rate": round(tracking_rate, 4),
        "imu": imu,
        "time_sync": time_sync,
        "warnings": warnings,
        "notes": notes,
    }
