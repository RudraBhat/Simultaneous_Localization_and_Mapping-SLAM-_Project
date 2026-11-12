from __future__ import annotations

import json
from pathlib import Path

import cv2

from .markers import generate_marker_image


def generate_markers(
    output_dir: Path,
    marker_ids: list[int],
    dictionary_name: str = "DICT_4X4_50",
    pixels: int = 900,
) -> list[Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    outputs: list[Path] = []
    for marker_id in marker_ids:
        image = generate_marker_image(dictionary_name, marker_id, pixels)
        path = output_dir / f"aruco_{dictionary_name.lower()}_id_{marker_id:03d}.png"
        if not cv2.imwrite(str(path), image):
            raise OSError(f"Could not write {path}")
        outputs.append(path)
    metadata = {
        "dictionary": dictionary_name,
        "marker_ids": marker_ids,
        "print_note": "Print all tags at one measured physical square size.",
    }
    (output_dir / "marker_set.json").write_text(
        json.dumps(metadata, indent=2) + "\n", encoding="utf-8"
    )
    return outputs
