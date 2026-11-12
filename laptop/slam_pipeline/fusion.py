from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation

from .geometry import apply_pose_error, invert_transform, pose_residual


@dataclass
class FilterUpdate:
    accepted: bool
    mahalanobis: float
    position_correction_m: float
    angle_correction_deg: float


class PoseCorrectionFilter:
    """Six-degree-of-freedom error-state EKF around a continuous VIO pose stream."""

    def __init__(self, initial_pose: np.ndarray):
        self.pose = initial_pose.copy()
        self.covariance = np.diag(
            [0.08**2, 0.08**2, 0.10**2, np.deg2rad(8) ** 2, np.deg2rad(8) ** 2, np.deg2rad(10) ** 2]
        )

    def predict(self, relative_vio_pose: np.ndarray, dt_s: float) -> None:
        distance = float(np.linalg.norm(relative_vio_pose[:3, 3]))
        angle = float(
            np.linalg.norm(Rotation.from_matrix(relative_vio_pose[:3, :3]).as_rotvec())
        )
        self.pose = self.pose @ relative_vio_pose
        position_sigma = 0.008 + 0.025 * distance + 0.004 * max(dt_s, 0.0)
        rotation_sigma = np.deg2rad(0.15 + 0.7 * np.rad2deg(angle))
        process_noise = np.diag(
            [position_sigma**2] * 3 + [rotation_sigma**2] * 3
        )
        self.covariance = self.covariance + process_noise

    def update(
        self,
        measurement_pose: np.ndarray,
        measurement_covariance: np.ndarray,
        gate_threshold: float = 60.0,
    ) -> FilterUpdate:
        residual = pose_residual(self.pose, measurement_pose)
        innovation_covariance = self.covariance + measurement_covariance
        solved = np.linalg.solve(innovation_covariance, residual)
        mahalanobis = float(residual @ solved)
        if not np.isfinite(mahalanobis) or mahalanobis > gate_threshold:
            return FilterUpdate(False, mahalanobis, 0.0, 0.0)

        gain = np.linalg.solve(innovation_covariance.T, self.covariance.T).T
        correction = gain @ residual
        self.pose = apply_pose_error(self.pose, correction)
        identity = np.eye(6)
        left = identity - gain
        self.covariance = left @ self.covariance @ left.T + gain @ measurement_covariance @ gain.T
        self.covariance = 0.5 * (self.covariance + self.covariance.T)
        return FilterUpdate(
            True,
            mahalanobis,
            float(np.linalg.norm(correction[:3])),
            float(np.rad2deg(np.linalg.norm(correction[3:]))),
        )


def marker_measurement_covariance(
    world_camera_pose: np.ndarray,
    marker_count: int,
    reprojection_rms_px: float,
    distance_m: float | None = None,
) -> np.ndarray:
    distance = float(distance_m) if distance_m is not None else float(np.linalg.norm(world_camera_pose[:3, 3]))
    pixel_error = max(0.0, float(reprojection_rms_px))
    independent_tags = max(1, marker_count)
    position_sigma = (0.015 + 0.004 * distance**2 + 0.002 * pixel_error) / np.sqrt(independent_tags)
    angle_sigma_deg = (1.0 + 0.45 * distance + 0.3 * pixel_error) / np.sqrt(independent_tags)
    return np.diag(
        [position_sigma**2] * 3 + [np.deg2rad(angle_sigma_deg) ** 2] * 3
    )


def aligned_baseline(
    arcore_poses: list[np.ndarray],
    anchor_index: int,
    anchor_world_camera_pose: np.ndarray,
) -> tuple[np.ndarray, list[np.ndarray]]:
    world_from_arcore = anchor_world_camera_pose @ invert_transform(arcore_poses[anchor_index])
    return world_from_arcore, [world_from_arcore @ pose for pose in arcore_poses]


def trajectory_errors(
    timestamps_ns: np.ndarray,
    estimated_positions: np.ndarray,
    ground_truth_rows: list[dict[str, float]],
) -> dict[str, float]:
    if len(ground_truth_rows) < 2:
        return {}
    gt_time = np.asarray([row["timestamp_ns"] for row in ground_truth_rows], dtype=np.int64)
    gt_xyz = np.asarray([[row["x"], row["y"], row["z"]] for row in ground_truth_rows], dtype=float)
    order = np.argsort(gt_time)
    gt_time = gt_time[order]
    gt_xyz = gt_xyz[order]
    valid = (timestamps_ns >= gt_time[0]) & (timestamps_ns <= gt_time[-1])
    if not np.any(valid):
        return {}
    query = timestamps_ns[valid]
    aligned_gt = np.column_stack(
        [np.interp(query, gt_time, gt_xyz[:, axis]) for axis in range(3)]
    )
    errors = np.linalg.norm(estimated_positions[valid] - aligned_gt, axis=1)
    travelled = float(np.sum(np.linalg.norm(np.diff(estimated_positions[valid], axis=0), axis=1)))
    return {
        "samples": int(len(errors)),
        "ate_rmse_m": float(np.sqrt(np.mean(errors**2))),
        "mean_position_error_m": float(np.mean(errors)),
        "endpoint_error_m": float(errors[-1]),
        "drift_per_m": float(errors[-1] / max(travelled, 1e-6)),
    }
