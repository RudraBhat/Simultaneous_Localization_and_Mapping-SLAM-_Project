from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation


# ARCore's physical camera frame is OpenGL-style (+x right, +y up, -z forward).
# OpenCV uses (+x right, +y down, +z forward). This proper rotation changes
# camera-coordinate convention without moving the camera origin.
ARCORE_FROM_OPENCV_CAMERA = np.diag([1.0, -1.0, -1.0, 1.0])


def pose_from_translation_quaternion(
    translation: tuple[float, float, float] | np.ndarray,
    quaternion_xyzw: tuple[float, float, float, float] | np.ndarray,
) -> np.ndarray:
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = Rotation.from_quat(np.asarray(quaternion_xyzw, dtype=float)).as_matrix()
    transform[:3, 3] = np.asarray(translation, dtype=float)
    return transform


def quaternion_xyzw(rotation: np.ndarray) -> np.ndarray:
    return Rotation.from_matrix(rotation[:3, :3]).as_quat()


def arcore_pose_to_opencv_camera(
    translation: tuple[float, float, float] | np.ndarray,
    quaternion_xyzw_value: tuple[float, float, float, float] | np.ndarray,
) -> np.ndarray:
    return pose_from_translation_quaternion(translation, quaternion_xyzw_value) @ ARCORE_FROM_OPENCV_CAMERA


def invert_transform(transform: np.ndarray) -> np.ndarray:
    rotation = transform[:3, :3]
    translation = transform[:3, 3]
    result = np.eye(4, dtype=float)
    result[:3, :3] = rotation.T
    result[:3, 3] = -rotation.T @ translation
    return result


def transform_points(transform: np.ndarray, points: np.ndarray) -> np.ndarray:
    points = np.asarray(points, dtype=float)
    return (transform[:3, :3] @ points.T).T + transform[:3, 3]


def pose_residual(reference: np.ndarray, measurement: np.ndarray) -> np.ndarray:
    position_error = measurement[:3, 3] - reference[:3, 3]
    rotation_error = Rotation.from_matrix(
        reference[:3, :3].T @ measurement[:3, :3]
    ).as_rotvec()
    return np.concatenate([position_error, rotation_error])


def apply_pose_error(transform: np.ndarray, error: np.ndarray) -> np.ndarray:
    updated = transform.copy()
    updated[:3, 3] += error[:3]
    updated[:3, :3] = updated[:3, :3] @ Rotation.from_rotvec(error[3:]).as_matrix()
    return updated


def pose_row(timestamp_ns: int, transform: np.ndarray, **extra: object) -> dict[str, object]:
    q = quaternion_xyzw(transform)
    row: dict[str, object] = {
        "timestamp_ns": int(timestamp_ns),
        "x": float(transform[0, 3]),
        "y": float(transform[1, 3]),
        "z": float(transform[2, 3]),
        "qx": float(q[0]),
        "qy": float(q[1]),
        "qz": float(q[2]),
        "qw": float(q[3]),
    }
    row.update(extra)
    return row


def rows_to_csv(path, rows: list[dict[str, object]]) -> None:
    import csv

    if not rows:
        path.write_text("")
        return
    fieldnames = list(rows[0].keys())
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
