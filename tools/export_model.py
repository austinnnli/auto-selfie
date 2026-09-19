#!/usr/bin/env python3
"""Export YOLO-pose to OpenVINO (P-3).

On a CPU-only laptop the PyTorch path runs at 6-8 fps and the target is 10+
sustained.  The export is a 2-3x speedup and it is the difference between a
control loop that works and one that lags half a second behind the world.

    python tools/export_model.py                 # OpenVINO, 480 input
    python tools/export_model.py --format ncnn
"""

from __future__ import annotations

import argparse
import sys


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--weights", default="yolo11n-pose.pt")
    parser.add_argument("--format", default="openvino", choices=["openvino", "ncnn", "onnx"])
    parser.add_argument("--imgsz", type=int, default=480)
    parser.add_argument("--half", action="store_true")
    args = parser.parse_args()

    try:
        from ultralytics import YOLO
    except ImportError:
        print("pip install -r requirements.txt first", file=sys.stderr)
        return 1

    model = YOLO(args.weights)
    path = model.export(format=args.format, imgsz=args.imgsz, half=args.half)
    print(f"exported to {path}")
    print("point config.json's model path at it, or pass it to PoseDetector(weights=...)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
