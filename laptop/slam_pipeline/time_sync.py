from __future__ import annotations

import csv
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from .session import Frame


def estimate_time_offset(session_dir: Path, frames: list[Frame]) -> dict[str, object]:
    imu_path = session_dir / "imu.csv"
    if not imu_path.exists():
        return {"status": "unavailable", "reason": "imu.csv is missing"}
    gyro_times: list[int] = []
    gyro_magnitudes: list[float] = []
    with imu_path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            if row.get("sensor") != "gyroscope":
                continue
            gyro_times.append(int(row["timestamp_ns"]))
            gyro_magnitudes.append(
                float(np.linalg.norm([float(row["x"]), float(row["y"]), float(row["z"])]))
            )
    if len(gyro_times) < 50:
        return {"status": "unavailable", "reason": "fewer than 50 gyro samples"}

    camera_times: list[int] = []
    camera_rates: list[float] = []
    active = [frame for frame in frames if frame.tracking_state.upper() == "TRACKING"]
    for previous, current in zip(active[:-1], active[1:]):
        dt = (current.timestamp_ns - previous.timestamp_ns) / 1e9
        if dt <= 0 or dt > 0.5:
            continue
        relative_rotation = previous.pose_arcore_cv[:3, :3].T @ current.pose_arcore_cv[:3, :3]
        rate = Rotation.from_matrix(relative_rotation).magnitude() / dt
        camera_times.append((previous.timestamp_ns + current.timestamp_ns) // 2)
        camera_rates.append(float(rate))
    if len(camera_times) < 12:
        return {"status": "unavailable", "reason": "not enough continuous camera poses"}

    gyro_t = np.asarray(gyro_times, dtype=np.float64) / 1e9
    gyro_rate = np.asarray(gyro_magnitudes, dtype=float)
    cam_t = np.asarray(camera_times, dtype=np.float64) / 1e9
    cam_rate = np.asarray(camera_rates, dtype=float)
    if np.std(gyro_rate) < 0.025 or np.std(cam_rate) < 0.025:
        return {
            "status": "insufficient_rotation",
            "reason": "turning motion is too small to estimate a camera-to-IMU time offset",
            "gyro_rate_std_rad_s": float(np.std(gyro_rate)),
            "camera_rate_std_rad_s": float(np.std(cam_rate)),
        }

    candidates = np.arange(-0.5, 0.5001, 0.002)
    best_score = -1.0
    best_offset = 0.0
    best_count = 0
    camera_centered = cam_rate - np.mean(cam_rate)
    camera_norm = float(np.linalg.norm(camera_centered))
    for offset in candidates:
        sample_times = cam_t + offset
        valid = (sample_times >= gyro_t[0]) & (sample_times <= gyro_t[-1])
        if int(np.sum(valid)) < 12:
            continue
        interpolated = np.interp(sample_times[valid], gyro_t, gyro_rate)
        gyro_centered = interpolated - np.mean(interpolated)
        denominator = camera_norm * float(np.linalg.norm(gyro_centered))
        if denominator <= 1e-9:
            continue
        score = float(camera_centered[valid] @ gyro_centered / denominator)
        if score > best_score:
            best_score = score
            best_offset = float(offset)
            best_count = int(np.sum(valid))

    if best_score < 0:
        return {"status": "unavailable", "reason": "camera and gyro streams do not overlap"}
    return {
        "status": "estimated",
        "imu_timestamp_offset_s": best_offset,
        "correlation": best_score,
        "matched_camera_intervals": best_count,
        "sign_convention": "Compare gyro(t_camera + offset) with camera angular speed; add offset to camera time to query IMU.",
        "warning": "Use as a diagnostic estimate only; sensor axes and phone VIO filtering can affect correlation.",
    }
