from __future__ import annotations

import csv
import json
import shutil
import zipfile
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .geometry import arcore_pose_to_opencv_camera


@dataclass
class Frame:
    index: int
    timestamp_ns: int
    image_path: Path
    tracking_state: str
    pose_arcore_cv: np.ndarray
    camera_matrix: np.ndarray
    width: int
    height: int


def locate_session(path: Path, extract_to: Path | None = None) -> Path:
    path = path.expanduser().resolve()
    if path.is_file() and zipfile.is_zipfile(path):
        if extract_to is None:
            extract_to = path.parent / (path.stem + "_extracted")
        extract_to = extract_to.resolve()
        extract_to.mkdir(parents=True, exist_ok=True)
        with zipfile.ZipFile(path) as archive:
            for member in archive.infolist():
                destination = (extract_to / member.filename).resolve()
                if extract_to not in destination.parents and destination != extract_to:
                    raise ValueError(f"Unsafe path in session ZIP: {member.filename}")
            archive.extractall(extract_to)
        path = extract_to
    if not path.exists():
        raise FileNotFoundError(path)
    if (path / "frames.csv").exists():
        return path
    candidates = sorted(path.rglob("frames.csv"))
    if not candidates:
        raise FileNotFoundError(f"No frames.csv found under {path}")
    return candidates[0].parent


def read_frames(session_dir: Path, calibration_path: Path | None = None) -> list[Frame]:
    calibration = None
    if calibration_path is not None:
        calibration = json.loads(calibration_path.read_text(encoding="utf-8"))
    frames: list[Frame] = []
    frames_file = session_dir / "frames.csv"
    with frames_file.open(newline="", encoding="utf-8") as stream:
        for index, row in enumerate(csv.DictReader(stream)):
            image_path = session_dir / row["image_file"]
            if not image_path.exists():
                continue
            if calibration is not None:
                # Avoid scaling the shared calibration matrix in place for each frame.
                matrix = np.asarray(calibration["camera_matrix"], dtype=float).copy()
                calibration_size = calibration.get("image_size")
                width = int(row.get("width", "0"))
                height = int(row.get("height", "0"))
                if calibration_size and width > 0 and height > 0:
                    scale_x = width / float(calibration_size[0])
                    scale_y = height / float(calibration_size[1])
                    matrix[0, :] *= scale_x
                    matrix[1, :] *= scale_y
            else:
                matrix = np.array(
                    [
                        [float(row["fx"]), 0.0, float(row["cx"])],
                        [0.0, float(row["fy"]), float(row["cy"])],
                        [0.0, 0.0, 1.0],
                    ],
                    dtype=float,
                )
            translation = [float(row[key]) for key in ("tx", "ty", "tz")]
            quaternion = [float(row[key]) for key in ("qx", "qy", "qz", "qw")]
            frames.append(
                Frame(
                    index=index,
                    timestamp_ns=int(row["timestamp_ns"]),
                    image_path=image_path,
                    tracking_state=row.get("tracking_state", "UNKNOWN"),
                    pose_arcore_cv=arcore_pose_to_opencv_camera(translation, quaternion),
                    camera_matrix=matrix,
                    width=int(row.get("width", "0")),
                    height=int(row.get("height", "0")),
                )
            )
    if not frames:
        raise ValueError("The session has no readable image frames.")
    return frames


def load_distortion(calibration_path: Path | None) -> np.ndarray:
    if calibration_path is None:
        return np.zeros((5, 1), dtype=float)
    calibration = json.loads(calibration_path.read_text(encoding="utf-8"))
    return np.asarray(calibration.get("distortion", [0, 0, 0, 0, 0]), dtype=float).reshape(-1, 1)


def read_imu_summary(session_dir: Path) -> dict[str, object]:
    imu_file = session_dir / "imu.csv"
    if not imu_file.exists():
        return {"samples": 0, "accelerometer_samples": 0, "gyroscope_samples": 0}
    counts = {"accelerometer": 0, "gyroscope": 0}
    first = None
    last = None
    with imu_file.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            timestamp = int(row["timestamp_ns"])
            first = timestamp if first is None else min(first, timestamp)
            last = timestamp if last is None else max(last, timestamp)
            label = row.get("sensor", "")
            if label in counts:
                counts[label] += 1
    duration = max(0.0, ((last or 0) - (first or 0)) / 1e9)
    return {
        "samples": sum(counts.values()),
        "accelerometer_samples": counts["accelerometer"],
        "gyroscope_samples": counts["gyroscope"],
        "duration_s": duration,
        "mean_accelerometer_hz": counts["accelerometer"] / duration if duration else 0.0,
        "mean_gyroscope_hz": counts["gyroscope"] / duration if duration else 0.0,
    }


def safe_extract_session(zip_path: Path, destination: Path) -> Path:
    if destination.exists():
        shutil.rmtree(destination)
    return locate_session(zip_path, destination)
