from __future__ import annotations

from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from .geometry import invert_transform
from .markers import ArucoDetector, MarkerMap, marker_object_corners
from .session import read_frames


def build_map_from_first_run(
    session_dir: Path,
    output_path: Path,
    dictionary_name: str = "DICT_4X4_50",
    marker_length_m: float = 0.16,
) -> MarkerMap:
    frames = read_frames(session_dir)
    detector = ArucoDetector(dictionary_name)
    observations: dict[int, list[np.ndarray]] = defaultdict(list)
    object_corners = marker_object_corners(marker_length_m)
    first_camera_pose = next(
        (frame.pose_arcore_cv for frame in frames if frame.tracking_state.upper() == "TRACKING"),
        None,
    )
    if first_camera_pose is None:
        raise ValueError("No ARCore-tracked frames were found in this session.")
    world_from_arcore = invert_transform(first_camera_pose)

    for frame in frames:
        if frame.tracking_state.upper() != "TRACKING":
            continue
        image = cv2.imread(str(frame.image_path), cv2.IMREAD_COLOR)
        if image is None:
            continue
        corners, ids, _ = detector.detect(image)
        if ids is None:
            continue
        world_camera = world_from_arcore @ frame.pose_arcore_cv
        for marker_index, marker_id_value in enumerate(ids.flatten()):
            image_points = np.asarray(corners[marker_index], dtype=np.float32).reshape(4, 2)
            success, rvec, tvec = cv2.solvePnP(
                object_corners,
                image_points,
                frame.camera_matrix,
                np.zeros((5, 1), dtype=float),
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
            if not success:
                continue
            rotation_camera_marker, _ = cv2.Rodrigues(rvec)
            camera_from_marker = np.eye(4, dtype=float)
            camera_from_marker[:3, :3] = rotation_camera_marker
            camera_from_marker[:3, 3] = np.asarray(tvec).reshape(3)
            observations[int(marker_id_value)].append(world_camera @ camera_from_marker)

    if not observations:
        raise ValueError("No ArUco markers were detected. Check the dictionary, IDs, and print size.")

    transforms: dict[int, np.ndarray] = {}
    for marker_id, poses in observations.items():
        positions = np.asarray([pose[:3, 3] for pose in poses])
        rotations = Rotation.from_matrix(np.asarray([pose[:3, :3] for pose in poses]))
        averaged = np.eye(4, dtype=float)
        averaged[:3, :3] = rotations.mean().as_matrix()
        averaged[:3, 3] = np.median(positions, axis=0)
        transforms[marker_id] = averaged

    marker_map = MarkerMap(
        dictionary_name=dictionary_name,
        marker_length_m=marker_length_m,
        transforms=transforms,
        world_frame="first_tracked_camera_frame",
    )
    marker_map.write(output_path)
    return marker_map
