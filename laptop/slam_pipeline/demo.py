from __future__ import annotations

import csv
import json
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from .geometry import ARCORE_FROM_OPENCV_CAMERA, invert_transform, transform_points
from .markers import MarkerMap, generate_marker_image, marker_object_corners


def create_synthetic_session(output_dir: Path, frame_count: int = 41) -> tuple[Path, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    session_dir = output_dir / "synthetic_session"
    images_dir = session_dir / "images"
    images_dir.mkdir(parents=True, exist_ok=True)

    dictionary_name = "DICT_4X4_50"
    marker_length = 0.20
    # Room W: x right, y along the floor into the room, z up.
    # The wall is at y=0; its printed marker +z faces back into the room (-y).
    marker_rotation = np.array(
        [[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]],
        dtype=float,
    )
    marker_transforms: dict[int, np.ndarray] = {}
    for marker_id, x in zip((10, 11, 12, 13), (-0.60, -0.20, 0.20, 0.60)):
        transform = np.eye(4, dtype=float)
        transform[:3, :3] = marker_rotation
        transform[:3, 3] = [x, 0.0, 1.2]
        marker_transforms[marker_id] = transform
    marker_map = MarkerMap(dictionary_name, marker_length, marker_transforms, "synthetic_room")
    map_path = output_dir / "synthetic_marker_map.json"
    marker_map.write(map_path)

    camera_matrix = np.array(
        [[620.0, 0.0, 320.0], [0.0, 620.0, 240.0], [0.0, 0.0, 1.0]],
        dtype=float,
    )
    distortion = np.zeros((5, 1), dtype=float)
    dictionary_id = getattr(cv2.aruco, dictionary_name)
    dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
    marker_images = {
        marker_id: generate_marker_image(dictionary_name, marker_id, 700)
        for marker_id in marker_transforms
    }
    local_corners = marker_object_corners(marker_length)
    frame_path = session_dir / "frames.csv"
    truth_path = session_dir / "ground_truth.csv"
    imu_path = session_dir / "imu.csv"
    forward_count = (frame_count + 1) // 2
    return_count = frame_count - forward_count + 1
    camera_rotation = np.array(
        [[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]],
        dtype=float,
    )
    positions = np.concatenate(
        [
            np.linspace(-0.38, 0.38, forward_count),
            np.linspace(0.38, -0.38, return_count)[1:],
        ]
    )

    with frame_path.open("w", newline="", encoding="utf-8") as frames_stream, \
            truth_path.open("w", newline="", encoding="utf-8") as truth_stream:
        frames_writer = csv.writer(frames_stream)
        truth_writer = csv.writer(truth_stream)
        frames_writer.writerow(
            [
                "timestamp_ns", "image_file", "tx", "ty", "tz", "qx", "qy", "qz", "qw",
                "tracking_state", "fx", "fy", "cx", "cy", "width", "height",
            ]
        )
        truth_writer.writerow(["timestamp_ns", "x", "y", "z"])
        for index, true_x in enumerate(positions):
            timestamp_ns = 1_000_000_000 + index * 100_000_000
            true_pose = np.eye(4, dtype=float)
            true_pose[:3, :3] = camera_rotation
            true_pose[:3, 3] = [true_x, -1.7, 1.2]
            reported_pose_cv = true_pose.copy()
            reported_pose_cv[0, 3] += 0.14 * index / max(1, frame_count - 1)
            reported_pose_ar = reported_pose_cv @ ARCORE_FROM_OPENCV_CAMERA
            q = Rotation.from_matrix(reported_pose_ar[:3, :3]).as_quat()
            filename = f"frame_{index + 1:06d}.jpg"
            image = np.full((480, 640, 3), 235, dtype=np.uint8)
            camera_from_world = invert_transform(true_pose)
            for marker_id, world_from_marker in marker_transforms.items():
                world_points = transform_points(world_from_marker, local_corners)
                camera_points = transform_points(camera_from_world, world_points)
                if np.any(camera_points[:, 2] <= 0.2):
                    continue
                projected, _ = cv2.projectPoints(
                    camera_points,
                    np.zeros(3),
                    np.zeros(3),
                    camera_matrix,
                    distortion,
                )
                destination = projected.reshape(4, 2).astype(np.float32)
                if (
                    np.max(destination[:, 0]) < 0
                    or np.min(destination[:, 0]) >= 640
                    or np.max(destination[:, 1]) < 0
                    or np.min(destination[:, 1]) >= 480
                ):
                    continue
                src = np.array(
                    [[0, 0], [699, 0], [699, 699], [0, 699]],
                    dtype=np.float32,
                )
                homography = cv2.getPerspectiveTransform(src, destination)
                warped = cv2.warpPerspective(
                    marker_images[marker_id],
                    homography,
                    (640, 480),
                    flags=cv2.INTER_LINEAR,
                    borderValue=235,
                )
                mask = np.zeros((480, 640), dtype=np.uint8)
                cv2.fillConvexPoly(mask, np.round(destination).astype(np.int32), 255)
                image[mask > 0] = cv2.cvtColor(warped, cv2.COLOR_GRAY2BGR)[mask > 0]
            cv2.imwrite(str(images_dir / filename), image)
            frames_writer.writerow(
                [
                    timestamp_ns,
                    "images/" + filename,
                    *[f"{value:.9g}" for value in reported_pose_ar[:3, 3]],
                    *[f"{value:.9g}" for value in q],
                    "TRACKING",
                    camera_matrix[0, 0],
                    camera_matrix[1, 1],
                    camera_matrix[0, 2],
                    camera_matrix[1, 2],
                    640,
                    480,
                ]
            )
            truth_writer.writerow([timestamp_ns, true_x, -1.7, 1.2])

    with imu_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["timestamp_ns", "sensor", "x", "y", "z", "accuracy"])
        for index in range(frame_count * 10):
            timestamp_ns = 1_000_000_000 + index * 10_000_000
            writer.writerow([timestamp_ns, "accelerometer", 0, 9.81, 0, 3])
            writer.writerow([timestamp_ns, "gyroscope", 0, 0, 0, 3])

    manifest = {
        "format_version": 1,
        "device": "synthetic demo",
        "camera_stream": "synthetic perspective-warped ArUco images",
        "camera_pose": "synthetic drifting VIO pose",
        "frame_time": "nanoseconds",
        "imu_time": "nanoseconds",
        "note": "For software demonstration only; this is not experimental data.",
    }
    (session_dir / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return session_dir, map_path
