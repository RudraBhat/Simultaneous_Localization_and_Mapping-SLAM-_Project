from __future__ import annotations

import argparse
import json
from pathlib import Path

from .calibration import calibrate_chessboard, generate_chessboard
from .diagnostics import inspect_session
from .demo import create_synthetic_session
from .generate_markers import generate_markers
from .map_builder import build_map_from_first_run
from .pipeline import process_session
from .session import locate_session


def _add_session_argument(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--session",
        type=Path,
        required=True,
        help="Android export ZIP or a directory containing frames.csv and images/.",
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="ie415-slam",
        description="Process Android phone camera, IMU, and ARCore data for the IE415 SLAM project.",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    process = subparsers.add_parser("process", help="Fuse ArUco map measurements with phone VIO poses.")
    _add_session_argument(process)
    process.add_argument("--map", type=Path, required=True, help="JSON surveyed marker map.")
    process.add_argument("--out", type=Path, required=True, help="Directory for trajectories and report.")
    process.add_argument("--calibration", type=Path, help="Optional OpenCV camera calibration JSON.")
    process.add_argument("--annotate-every", type=int, default=10, help="Save an overlay every N frames.")

    inspect = subparsers.add_parser("inspect", help="Check a phone recording before running SLAM.")
    _add_session_argument(inspect)
    inspect.add_argument(
        "--workdir",
        type=Path,
        default=Path("../work/inspect"),
        help="Folder used to extract a ZIP for inspection (default: ../work/inspect).",
    )

    calibrate = subparsers.add_parser("calibrate", help="Estimate camera intrinsics from chessboard images.")
    calibrate.add_argument("--images", type=Path, required=True, help="Folder containing calibration images.")
    calibrate.add_argument("--out", type=Path, required=True, help="Output camera calibration JSON.")
    calibrate.add_argument("--columns", type=int, default=9, help="Chessboard inner-corner columns.")
    calibrate.add_argument("--rows", type=int, default=6, help="Chessboard inner-corner rows.")
    calibrate.add_argument("--square-size-m", type=float, default=0.024, help="Printed square width in metres.")

    board = subparsers.add_parser("make-board", help="Create a printable chessboard calibration target.")
    board.add_argument("--out", type=Path, required=True, help="Output PNG.")
    board.add_argument("--columns", type=int, default=10, help="Number of printed squares across.")
    board.add_argument("--rows", type=int, default=7, help="Number of printed squares down.")
    board.add_argument("--square-pixels", type=int, default=120)

    markers = subparsers.add_parser("make-markers", help="Generate printable ArUco tag images.")
    markers.add_argument("--out", type=Path, required=True, help="Output folder.")
    markers.add_argument("--ids", type=int, nargs="+", required=True)
    markers.add_argument("--dictionary", default="DICT_4X4_50")
    markers.add_argument("--pixels", type=int, default=900)

    map_parser = subparsers.add_parser(
        "build-map", help="Create an initial persistent map from an exploratory phone run."
    )
    _add_session_argument(map_parser)
    map_parser.add_argument("--out", type=Path, required=True, help="Output marker-map JSON.")
    map_parser.add_argument("--dictionary", default="DICT_4X4_50")
    map_parser.add_argument("--marker-length-m", type=float, default=0.16)

    demo = subparsers.add_parser("demo", help="Generate and process a synthetic end-to-end example.")
    demo.add_argument("--out", type=Path, required=True, help="Output folder for demo and report.")
    demo.add_argument("--frames", type=int, default=41)

    return parser


def _session_directory(session_path: Path, output_dir: Path) -> Path:
    if session_path.is_file():
        return locate_session(session_path, output_dir / "_imported_session")
    return locate_session(session_path)


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        if args.command == "process":
            args.out.mkdir(parents=True, exist_ok=True)
            session_dir = _session_directory(args.session, args.out)
            summary = process_session(
                session_dir,
                args.map,
                args.out,
                calibration_path=args.calibration,
                annotate_every=args.annotate_every,
            )
            print(json.dumps(summary, indent=2))
            print("Open " + str(args.out / "index.html") + " to view the run report.")
            return 0

        if args.command == "inspect":
            session_dir = _session_directory(args.session, args.workdir)
            summary = inspect_session(session_dir)
            print(json.dumps(summary, indent=2))
            for note in summary["notes"]:
                print("Note: " + str(note))
            if summary["warnings"]:
                print("Review the warnings before processing this run.")
            else:
                print("The recording passed the basic capture checks.")
            return 0

        if args.command == "calibrate":
            images = sorted(
                path
                for path in args.images.iterdir()
                if path.suffix.lower() in {".jpg", ".jpeg", ".png", ".bmp"}
            )
            result = calibrate_chessboard(
                images,
                args.out,
                args.columns,
                args.rows,
                args.square_size_m,
            )
            print(json.dumps(result, indent=2))
            return 0

        if args.command == "make-board":
            generate_chessboard(args.out, args.columns, args.rows, args.square_pixels)
            print("Wrote " + str(args.out))
            return 0

        if args.command == "make-markers":
            paths = generate_markers(args.out, args.ids, args.dictionary, args.pixels)
            print("\n".join(str(path) for path in paths))
            return 0

        if args.command == "build-map":
            session_dir = _session_directory(args.session, args.out.parent)
            marker_map = build_map_from_first_run(
                session_dir,
                args.out,
                args.dictionary,
                args.marker_length_m,
            )
            print(f"Wrote {len(marker_map.transforms)} markers to {args.out}")
            print("The map origin is the first tracked camera frame; survey marker poses for measured accuracy.")
            return 0

        if args.command == "demo":
            demo_dir = args.out / "synthetic"
            session_dir, map_path = create_synthetic_session(demo_dir, args.frames)
            report_dir = args.out / "report"
            summary = process_session(session_dir, map_path, report_dir)
            print(json.dumps(summary, indent=2))
            print("Open " + str(report_dir / "index.html") + " to view the synthetic demo.")
            return 0

        raise RuntimeError("Unknown command.")
    except Exception as error:
        print("Error: " + str(error))
        return 2
