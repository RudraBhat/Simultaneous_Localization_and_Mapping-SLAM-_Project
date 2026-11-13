from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from .geometry import invert_transform, transform_points


@dataclass
class MarkerMap:
    dictionary_name: str
    marker_length_m: float
    transforms: dict[int, np.ndarray]
    world_frame: str = "surveyed_room"

    @classmethod
    def load(cls, path: Path) -> "MarkerMap":
        raw = json.loads(path.read_text(encoding="utf-8"))
        marker_length_m = float(raw["marker_length_m"])
        if not np.isfinite(marker_length_m) or marker_length_m <= 0:
            raise ValueError("marker_length_m must be a positive finite value in metres.")
        transforms: dict[int, np.ndarray] = {}
        for marker in raw.get("markers", []):
            marker_id = int(marker["id"])
            if marker_id in transforms:
                raise ValueError(f"Marker ID {marker_id} appears more than once in the map.")
            transform = np.asarray(marker["T_world_marker"], dtype=float)
            if transform.shape != (4, 4):
                raise ValueError(f"Marker {marker['id']} must have a 4x4 T_world_marker.")
            if not np.all(np.isfinite(transform)):
                raise ValueError(f"Marker {marker_id} has non-finite transform values.")
            if not np.allclose(transform[3], [0.0, 0.0, 0.0, 1.0], atol=1e-6):
                raise ValueError(f"Marker {marker_id} has an invalid homogeneous-transform last row.")
            rotation = transform[:3, :3]
            if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-3) or not np.isclose(
                np.linalg.det(rotation), 1.0, atol=1e-3
            ):
                raise ValueError(f"Marker {marker_id} rotation must be a proper orthonormal matrix.")
            transforms[marker_id] = transform
        if not transforms:
            raise ValueError("The marker map has no known markers.")
        return cls(
            dictionary_name=raw.get("dictionary", "DICT_4X4_50"),
            marker_length_m=marker_length_m,
            transforms=transforms,
            world_frame=raw.get("world_frame", "surveyed_room"),
        )

    def write(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        markers = [
            {"id": marker_id, "T_world_marker": transform.tolist()}
            for marker_id, transform in sorted(self.transforms.items())
        ]
        path.write_text(
            json.dumps(
                {
                    "format_version": 1,
                    "dictionary": self.dictionary_name,
                    "marker_length_m": self.marker_length_m,
                    "world_frame": self.world_frame,
                    "markers": markers,
                },
                indent=2,
            )
            + "\n",
            encoding="utf-8",
        )


class ArucoDetector:
    def __init__(self, dictionary_name: str):
        if not hasattr(cv2, "aruco"):
            raise RuntimeError(
                "OpenCV ArUco is unavailable. Install opencv-contrib-python-headless."
            )
        dictionary_id = getattr(cv2.aruco, dictionary_name, None)
        if dictionary_id is None:
            raise ValueError(f"Unknown OpenCV ArUco dictionary: {dictionary_name}")
        self.dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        self.parameters = cv2.aruco.DetectorParameters()
        if hasattr(cv2.aruco, "ArucoDetector"):
            self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)
        else:
            self.detector = None

    def detect(self, image: np.ndarray):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        if self.detector is not None:
            corners, ids, rejected = self.detector.detectMarkers(gray)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(
                gray, self.dictionary, parameters=self.parameters
            )
        return corners, ids, rejected


def marker_object_corners(size_m: float) -> np.ndarray:
    half = size_m / 2.0
    # Marker-local +x points right, +y points to the printed top edge,
    # and +z points out of the printed face toward the observer.
    return np.array(
        [
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
            [-half, -half, 0.0],
        ],
        dtype=np.float32,
    )


def estimate_world_camera_pose(
    image: np.ndarray,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
    marker_map: MarkerMap,
    detector: ArucoDetector,
) -> tuple[np.ndarray | None, dict[str, object], np.ndarray]:
    corners, ids, _ = detector.detect(image)
    annotated = image.copy()
    ids_list = [] if ids is None else [int(value) for value in ids.flatten()]
    if ids is not None and len(ids):
        cv2.aruco.drawDetectedMarkers(annotated, corners, ids)

    object_points: list[np.ndarray] = []
    image_points: list[np.ndarray] = []
    used_ids: list[int] = []
    relative_poses: list[dict[str, object]] = []
    local_corners = marker_object_corners(marker_map.marker_length_m)
    for index, marker_id in enumerate(ids_list):
        transform_world_marker = marker_map.transforms.get(marker_id)
        if transform_world_marker is None:
            continue
        world_corners = transform_points(transform_world_marker, local_corners)
        image_corner_set = np.asarray(corners[index], dtype=np.float32).reshape(4, 2)
        object_points.extend(world_corners)
        image_points.extend(image_corner_set)
        used_ids.append(marker_id)
        try:
            solved, marker_rvec, marker_tvec = cv2.solvePnP(
                local_corners,
                image_corner_set,
                camera_matrix,
                distortion,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
            if solved:
                marker_rotation, _ = cv2.Rodrigues(marker_rvec)
                camera_from_marker = np.eye(4, dtype=float)
                camera_from_marker[:3, :3] = marker_rotation
                camera_from_marker[:3, 3] = np.asarray(marker_tvec).reshape(3)
                relative_poses.append(
                    {"id": marker_id, "T_camera_marker": camera_from_marker.tolist()}
                )
                cv2.drawFrameAxes(
                    annotated,
                    camera_matrix,
                    distortion,
                    marker_rvec,
                    marker_tvec,
                    min(0.06, marker_map.marker_length_m * 0.3),
                )
        except cv2.error:
            # Preserve the multi-marker solve if one diagnostic pose is degenerate.
            pass

    if len(used_ids) == 0:
        return None, {
            "marker_ids": ids_list,
            "used_ids": [],
            "relative_marker_poses": [],
            "reprojection_rms_px": "",
            "inliers": 0,
            "reason": "no mapped marker",
        }, annotated

    obj = np.asarray(object_points, dtype=np.float32)
    img = np.asarray(image_points, dtype=np.float32)
    try:
        success, rvec, tvec, inliers = cv2.solvePnPRansac(
            obj,
            img,
            camera_matrix,
            distortion,
            iterationsCount=150,
            reprojectionError=4.0,
            confidence=0.995,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
    except cv2.error:
        success, rvec, tvec = cv2.solvePnP(
            obj, img, camera_matrix, distortion, flags=cv2.SOLVEPNP_EPNP
        )
        inliers = np.arange(len(obj), dtype=np.int32).reshape(-1, 1) if success else None

    if not success:
        return None, {
            "marker_ids": ids_list,
            "used_ids": used_ids,
            "relative_marker_poses": relative_poses,
            "reprojection_rms_px": "",
            "inliers": 0,
            "reason": "solvePnP failed",
        }, annotated

    rotation_camera_world, _ = cv2.Rodrigues(rvec)
    transform_camera_world = np.eye(4, dtype=float)
    transform_camera_world[:3, :3] = rotation_camera_world
    transform_camera_world[:3, 3] = np.asarray(tvec).reshape(3)
    transform_world_camera = invert_transform(transform_camera_world)

    projected, _ = cv2.projectPoints(obj, rvec, tvec, camera_matrix, distortion)
    error = projected.reshape(-1, 2) - img
    reprojection_rms = float(np.sqrt(np.mean(np.sum(error * error, axis=1))))
    inlier_count = len(inliers) if inliers is not None else len(obj)
    marker_distances = [
        float(np.linalg.norm(transform[:3, 3] - transform_world_camera[:3, 3]))
        for marker_id in used_ids
        if (transform := marker_map.transforms.get(marker_id)) is not None
    ]
    report = {
        "marker_ids": ids_list,
        "used_ids": used_ids,
        "relative_marker_poses": relative_poses,
        "reprojection_rms_px": reprojection_rms,
        "inliers": inlier_count,
        "mean_marker_distance_m": float(np.mean(marker_distances)) if marker_distances else 0.0,
        "reason": "ok",
    }

    return transform_world_camera, report, annotated


def generate_marker_image(dictionary_name: str, marker_id: int, pixels: int = 700) -> np.ndarray:
    dictionary_id = getattr(cv2.aruco, dictionary_name)
    dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
    if hasattr(cv2.aruco, "generateImageMarker"):
        return cv2.aruco.generateImageMarker(dictionary, int(marker_id), pixels)
    result = np.zeros((pixels, pixels), dtype=np.uint8)
    cv2.aruco.drawMarker(dictionary, int(marker_id), pixels, result, 1)
    return result
