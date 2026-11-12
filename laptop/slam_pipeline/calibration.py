from __future__ import annotations

import json
from pathlib import Path

import cv2
import numpy as np


def calibrate_chessboard(
    images: list[Path],
    output: Path,
    columns: int = 9,
    rows: int = 6,
    square_size_m: float = 0.024,
) -> dict[str, object]:
    if columns < 3 or rows < 3 or square_size_m <= 0:
        raise ValueError("Use at least a 3x3 inner-corner pattern and a positive square size.")
    object_template = np.zeros((columns * rows, 3), dtype=np.float32)
    object_template[:, :2] = np.mgrid[0:columns, 0:rows].T.reshape(-1, 2)
    object_template *= square_size_m
    object_points: list[np.ndarray] = []
    image_points: list[np.ndarray] = []
    image_size: tuple[int, int] | None = None
    used_images: list[str] = []
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001)

    for path in images:
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            continue
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        current_size = (gray.shape[1], gray.shape[0])
        if image_size is not None and current_size != image_size:
            raise ValueError("All calibration images must use the same resolution and camera lens.")
        image_size = current_size
        found, corners = cv2.findChessboardCorners(
            gray, (columns, rows),
            cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE,
        )
        if not found:
            continue
        refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        object_points.append(object_template.copy())
        image_points.append(refined)
        used_images.append(str(path))

    if image_size is None or len(object_points) < 8:
        raise ValueError(
            f"Found corners in {len(object_points)} images. Capture at least 8 sharp, varied board views."
        )

    rms, camera_matrix, distortion, _, _ = cv2.calibrateCamera(
        object_points, image_points, image_size, None, None
    )
    result: dict[str, object] = {
        "format_version": 1,
        "camera_matrix": camera_matrix.tolist(),
        "distortion": distortion.reshape(-1).tolist(),
        "image_size": list(image_size),
        "chessboard_inner_corners": [columns, rows],
        "square_size_m": square_size_m,
        "reprojection_rms_px": float(rms),
        "calibration_images": used_images,
        "note": "Use the same phone camera, resolution, focus, and image rotation used by the capture app.",
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    return result


def generate_chessboard(
    output: Path,
    columns: int = 10,
    rows: int = 7,
    square_pixels: int = 120,
) -> Path:
    board = np.zeros((rows * square_pixels, columns * square_pixels), dtype=np.uint8)
    for row in range(rows):
        for column in range(columns):
            color = 255 if (row + column) % 2 == 0 else 0
            board[
                row * square_pixels : (row + 1) * square_pixels,
                column * square_pixels : (column + 1) * square_pixels,
            ] = color
    output.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output), board)
    return output
