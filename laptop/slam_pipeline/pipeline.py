from __future__ import annotations

import csv
import json
import math
import time
from pathlib import Path

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from .fusion import (
    PoseCorrectionFilter,
    aligned_baseline,
    marker_measurement_covariance,
    trajectory_errors,
)
from .geometry import invert_transform, pose_row
from .markers import ArucoDetector, MarkerMap, estimate_world_camera_pose
from .session import Frame, read_frames, read_imu_summary
from .time_sync import estimate_time_offset


def _read_ground_truth(path: Path) -> list[dict[str, float]]:
    rows: list[dict[str, float]] = []
    with path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            timestamp = row.get("timestamp_ns") or row.get("time_ns")
            if timestamp is None and row.get("time_s") is not None:
                timestamp_ns = int(float(row["time_s"]) * 1e9)
            else:
                timestamp_ns = int(float(timestamp))
            rows.append(
                {
                    "timestamp_ns": timestamp_ns,
                    "x": float(row["x"]),
                    "y": float(row["y"]),
                    "z": float(row["z"]),
                }
            )
    return rows


def _write_observations(path: Path, rows: list[dict[str, object]]) -> None:
    fields = [
        "timestamp_ns",
        "detected_ids",
        "mapped_ids",
        "relative_marker_poses_json",
        "reprojection_rms_px",
        "inliers",
        "filter_update",
        "mahalanobis",
        "position_correction_m",
        "angle_correction_deg",
        "reason",
    ]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def _plot_trajectory(
    output_dir: Path,
    baseline: list[np.ndarray],
    corrected: list[np.ndarray],
    marker_map: MarkerMap,
    ground_truth: list[dict[str, float]],
) -> None:
    baseline_xyz = np.asarray([pose[:3, 3] for pose in baseline])
    corrected_xyz = np.asarray([pose[:3, 3] for pose in corrected])

    fig, ax = plt.subplots(figsize=(9, 7), constrained_layout=True)
    ax.plot(baseline_xyz[:, 0], baseline_xyz[:, 1], color="#8291A5", lw=1.8, label="VIO baseline")
    ax.plot(corrected_xyz[:, 0], corrected_xyz[:, 1], color="#087E8B", lw=2.4, label="ArUco corrected")
    if ground_truth:
        truth = np.asarray([[row["x"], row["y"]] for row in ground_truth])
        ax.plot(truth[:, 0], truth[:, 1], "k--", lw=1.4, label="Reference")
    marker_xyz = np.asarray([pose[:3, 3] for pose in marker_map.transforms.values()])
    ax.scatter(marker_xyz[:, 0], marker_xyz[:, 1], marker="s", s=70, color="#E07A5F", label="Mapped markers")
    for marker_id, transform in marker_map.transforms.items():
        ax.annotate(str(marker_id), transform[:2, 3], xytext=(5, 4), textcoords="offset points")
    ax.set_title("World x-y projection")
    ax.set_xlabel("World x (m)")
    ax.set_ylabel("World y (m)")
    ax.axis("equal")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best")
    fig.savefig(output_dir / "trajectory_xy.png", dpi=160)
    plt.close(fig)

    fig = plt.figure(figsize=(10, 7), constrained_layout=True)
    ax3 = fig.add_subplot(111, projection="3d")
    ax3.plot(baseline_xyz[:, 0], baseline_xyz[:, 1], baseline_xyz[:, 2], color="#8291A5", label="VIO baseline")
    ax3.plot(corrected_xyz[:, 0], corrected_xyz[:, 1], corrected_xyz[:, 2], color="#087E8B", label="ArUco corrected")
    ax3.scatter(marker_xyz[:, 0], marker_xyz[:, 1], marker_xyz[:, 2], marker="s", s=45, color="#E07A5F", label="Markers")
    ax3.set_title("Camera trajectory and marker map")
    ax3.set_xlabel("World x (m)")
    ax3.set_ylabel("World y (m)")
    ax3.set_zlabel("World z (m)")
    ax3.legend(loc="best")
    fig.savefig(output_dir / "trajectory_3d.png", dpi=160)
    plt.close(fig)


def _write_html(output_dir: Path, summary: dict[str, object]) -> None:
    summary_rows = []
    for key, value in summary.items():
        if isinstance(value, dict):
            for subkey, subvalue in value.items():
                summary_rows.append(f"<tr><th>{key}.{subkey}</th><td>{subvalue}</td></tr>")
        else:
            summary_rows.append(f"<tr><th>{key}</th><td>{value}</td></tr>")
    html = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>IE415 SLAM run report</title>
<style>
body{font:16px/1.5 system-ui,sans-serif;max-width:1100px;margin:2rem auto;padding:0 1rem;color:#1c2633}
h1,h2{color:#12334a}table{border-collapse:collapse;width:100%;margin:1rem 0 2rem}
th,td{text-align:left;border-bottom:1px solid #d9e0e6;padding:.45rem .7rem}
th{width:35%;font-weight:600}img{width:100%;height:auto;border:1px solid #d9e0e6;border-radius:8px}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:1rem}
@media(max-width:700px){.grid{grid-template-columns:1fr}}
</style>
</head>
<body>
<h1>IE415 phone-laptop SLAM run</h1>
<p>Compare the continuous phone VIO estimate with the estimate corrected by surveyed ArUco markers.</p>
<table>""" + "".join(summary_rows) + """</table>
<div class="grid">
<section><h2>Top-down view</h2><img src="trajectory_xy.png" alt="Top-down trajectory comparison"></section>
<section><h2>3D view</h2><img src="trajectory_3d.png" alt="Three-dimensional trajectory and marker map"></section>
</div>
<p>Per-frame outputs are in trajectory_baseline.csv, trajectory_corrected.csv, and marker_observations.csv.</p>
</body></html>
"""
    (output_dir / "index.html").write_text(html, encoding="utf-8")


def process_session(
    session_dir: Path,
    marker_map_path: Path,
    output_dir: Path,
    calibration_path: Path | None = None,
    annotate_every: int = 10,
) -> dict[str, object]:
    processing_started = time.perf_counter()
    session_dir = session_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    marker_map = MarkerMap.load(marker_map_path)
    frames = read_frames(session_dir, calibration_path)
    detector = ArucoDetector(marker_map.dictionary_name)
    distortion = (
        np.zeros((5, 1), dtype=float)
        if calibration_path is None
        else np.asarray(
            json.loads(calibration_path.read_text(encoding="utf-8")).get(
                "distortion", [0, 0, 0, 0, 0]
            ),
            dtype=float,
        ).reshape(-1, 1)
    )

    measurements: list[np.ndarray | None] = []
    reports: list[dict[str, object]] = []
    annotated_dir = output_dir / "annotated"
    annotated_dir.mkdir(exist_ok=True)
    for frame_index, frame in enumerate(frames):
        image = cv2.imread(str(frame.image_path), cv2.IMREAD_COLOR)
        if image is None:
            measurements.append(None)
            reports.append(
                {
                    "marker_ids": [],
                    "used_ids": [],
                    "reprojection_rms_px": "",
                    "inliers": 0,
                    "reason": "image could not be read",
                }
            )
            continue
        measurement, report, annotated = estimate_world_camera_pose(
            image,
            frame.camera_matrix,
            distortion,
            marker_map,
            detector,
        )
        measurements.append(measurement)
        reports.append(report)
        if annotate_every > 0 and frame_index % annotate_every == 0:
            cv2.putText(
                annotated,
                "tracking: " + frame.tracking_state,
                (12, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (30, 160, 40),
                2,
                cv2.LINE_AA,
            )
            cv2.imwrite(str(annotated_dir / f"frame_{frame_index:06d}.jpg"), annotated)

    anchor_index = next(
        (
            index
            for index, (frame, measurement) in enumerate(zip(frames, measurements))
            if measurement is not None and frame.tracking_state.upper() == "TRACKING"
        ),
        None,
    )
    if anchor_index is None:
        raise ValueError(
            "No mapped ArUco marker was solved while ARCore tracking was active. "
            "Check marker_map.json, printed marker IDs, calibration, and lighting."
        )

    world_from_arcore, baseline = aligned_baseline(
        [frame.pose_arcore_cv for frame in frames],
        anchor_index,
        measurements[anchor_index],
    )
    first_tracking_index = next(
        (i for i, frame in enumerate(frames) if frame.tracking_state.upper() == "TRACKING"),
        anchor_index,
    )
    filter_state = PoseCorrectionFilter(baseline[first_tracking_index])
    corrected: list[np.ndarray] = []
    update_rows: list[dict[str, object]] = []
    previous_index = first_tracking_index
    previous_timestamp = frames[first_tracking_index].timestamp_ns
    marker_updates_accepted = 0
    marker_updates_rejected = 0
    tracking_gap = False

    for index, frame in enumerate(frames):
        measurement = measurements[index]
        report = reports[index]
        if index < first_tracking_index:
            corrected.append(baseline[index].copy())
            update_rows.append(_report_row(frame, report, None, "not initialized"))
            continue
        if frame.tracking_state.upper() != "TRACKING":
            corrected.append(filter_state.pose.copy())
            update_rows.append(_report_row(frame, report, None, "ARCore tracking inactive"))
            tracking_gap = True
            continue
        relocalized = False
        if index > first_tracking_index:
            if tracking_gap:
                if measurement is not None:
                    filter_state = PoseCorrectionFilter(measurement)
                    marker_updates_accepted += 1
                    relocalized = True
                tracking_gap = False
            else:
                previous_pose = frames[previous_index].pose_arcore_cv
                current_pose = frame.pose_arcore_cv
                relative = invert_transform(previous_pose) @ current_pose
                dt_s = max(0.0, (frame.timestamp_ns - previous_timestamp) / 1e9)
                delta_distance = float(np.linalg.norm(relative[:3, 3]))
                delta_angle = float(np.linalg.norm(cv2.Rodrigues(relative[:3, :3])[0]))
                if delta_distance < 1.5 and delta_angle < math.radians(100):
                    filter_state.predict(relative, dt_s)
            previous_index = index
            previous_timestamp = frame.timestamp_ns

        update = None
        if measurement is not None and not relocalized:
            used_count = max(1, len(report.get("used_ids", [])))
            rms = float(report.get("reprojection_rms_px") or 0.0)
            covariance = marker_measurement_covariance(
                measurement,
                used_count,
                rms,
                float(report.get("mean_marker_distance_m") or 0.0),
            )
            update = filter_state.update(measurement, covariance)
            if update.accepted:
                marker_updates_accepted += 1
            else:
                marker_updates_rejected += 1
        corrected.append(filter_state.pose.copy())
        update_rows.append(
            _report_row(frame, report, update, "relocalized from marker" if relocalized else "no mapped marker")
        )

    baseline_rows = [
        pose_row(frame.timestamp_ns, pose, tracking_state=frame.tracking_state)
        for frame, pose in zip(frames, baseline)
    ]
    corrected_rows = [
        pose_row(frame.timestamp_ns, pose, tracking_state=frame.tracking_state)
        for frame, pose in zip(frames, corrected)
    ]
    from .geometry import rows_to_csv

    rows_to_csv(output_dir / "trajectory_baseline.csv", baseline_rows)
    rows_to_csv(output_dir / "trajectory_corrected.csv", corrected_rows)
    _write_observations(output_dir / "marker_observations.csv", update_rows)

    ground_truth_path = session_dir / "ground_truth.csv"
    ground_truth = _read_ground_truth(ground_truth_path) if ground_truth_path.exists() else []
    timestamps = np.asarray([frame.timestamp_ns for frame in frames], dtype=np.int64)
    baseline_xyz = np.asarray([pose[:3, 3] for pose in baseline])
    corrected_xyz = np.asarray([pose[:3, 3] for pose in corrected])
    metrics: dict[str, object] = {
        "baseline": trajectory_errors(timestamps, baseline_xyz, ground_truth),
        "corrected": trajectory_errors(timestamps, corrected_xyz, ground_truth),
    }

    imu_summary = read_imu_summary(session_dir)
    time_sync = estimate_time_offset(session_dir, frames)
    observed_frames = sum(1 for report in reports if report.get("used_ids"))
    tracking_frames = sum(1 for frame in frames if frame.tracking_state.upper() == "TRACKING")
    duration_s = max((frames[-1].timestamp_ns - frames[0].timestamp_ns) / 1e9, 0.0)
    estimation_runtime_s = time.perf_counter() - processing_started
    summary: dict[str, object] = {
        "session": session_dir.name,
        "frames": len(frames),
        "duration_s": round(duration_s, 3),
        "arcore_tracking_rate": round(tracking_frames / len(frames), 4),
        "marker_observed_frames": observed_frames,
        "mapped_marker_observation_rate": round(observed_frames / len(frames), 4),
        "first_anchor_frame": anchor_index,
        "marker_updates_accepted": marker_updates_accepted,
        "marker_updates_rejected": marker_updates_rejected,
        "mapped_markers": len(marker_map.transforms),
        "imu": imu_summary,
        "time_sync": time_sync,
        "estimation_runtime_s": round(estimation_runtime_s, 3),
        "estimation_processing_fps": round(len(frames) / max(estimation_runtime_s, 1e-9), 2),
        "trajectory_metrics": metrics,
        "world_from_arcore": world_from_arcore.tolist(),
        "calibration_source": str(calibration_path) if calibration_path else "ARCore image intrinsics; zero distortion",
        "note": "Trajectory alignment uses the first valid mapped-marker observation; physical accuracy depends on map survey quality.",
    }
    (output_dir / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    (output_dir / "time_sync.json").write_text(json.dumps(time_sync, indent=2) + "\n", encoding="utf-8")
    _plot_trajectory(output_dir, baseline, corrected, marker_map, ground_truth)
    _write_html(output_dir, summary)
    return summary


def _report_row(
    frame: Frame,
    report: dict[str, object],
    update,
    fallback_reason: str,
) -> dict[str, object]:
    return {
        "timestamp_ns": frame.timestamp_ns,
        "detected_ids": ";".join(str(value) for value in report.get("marker_ids", [])),
        "mapped_ids": ";".join(str(value) for value in report.get("used_ids", [])),
        "relative_marker_poses_json": json.dumps(
            report.get("relative_marker_poses", []), separators=(",", ":")
        ),
        "reprojection_rms_px": report.get("reprojection_rms_px", ""),
        "inliers": report.get("inliers", 0),
        "filter_update": "" if update is None else update.accepted,
        "mahalanobis": "" if update is None else update.mahalanobis,
        "position_correction_m": "" if update is None else update.position_correction_m,
        "angle_correction_deg": "" if update is None else update.angle_correction_deg,
        "reason": report.get("reason") or fallback_reason,
    }
