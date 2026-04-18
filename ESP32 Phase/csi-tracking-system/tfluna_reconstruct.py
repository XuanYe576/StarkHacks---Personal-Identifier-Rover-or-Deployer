#!/usr/bin/env python3
"""
Capture TF-Luna UART data and reconstruct maximum-resolution point cloud.

Example:
  python tfluna_reconstruct.py --port /dev/ttyUSB0 --seconds 8 --out cloud.ply
"""

from __future__ import annotations

import argparse
from pathlib import Path

import pandas as pd

from src.tfluna_lidar import capture_sweep, reconstruct_point_cloud, save_ply_xyz


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="TF-Luna max-resolution reconstruction")
    p.add_argument("--port", required=True, help="Serial port (e.g. /dev/ttyUSB0 or COM4)")
    p.add_argument("--baudrate", type=int, default=115200, help="TF-Luna UART baudrate")
    p.add_argument("--seconds", type=float, default=8.0, help="Capture duration")
    p.add_argument("--azimuth-min", type=float, default=-90.0, help="Sweep azimuth min degree")
    p.add_argument("--azimuth-max", type=float, default=90.0, help="Sweep azimuth max degree")
    p.add_argument("--elevation", type=float, default=0.0, help="Sweep elevation degree")
    p.add_argument("--strength-min", type=int, default=20, help="Minimum TF-Luna strength")
    p.add_argument("--out", default="tfluna_cloud.ply", help="Output PLY path")
    p.add_argument("--csv", default="tfluna_capture.csv", help="Output CSV path")
    return p.parse_args()


def main() -> None:
    args = parse_args()

    samples = capture_sweep(
        port=args.port,
        baudrate=args.baudrate,
        seconds=args.seconds,
        azimuth_min_deg=args.azimuth_min,
        azimuth_max_deg=args.azimuth_max,
        elevation_deg=args.elevation,
    )
    if not samples:
        raise RuntimeError("No TF-Luna samples captured.")

    df = pd.DataFrame(
        {
            "ts": [s.ts for s in samples],
            "distance_m": [s.distance_m for s in samples],
            "strength": [s.strength for s in samples],
            "temp_c": [s.temp_c for s in samples],
            "azimuth_deg": [s.azimuth_deg for s in samples],
            "elevation_deg": [s.elevation_deg for s in samples],
        }
    )
    df.to_csv(args.csv, index=False)

    cloud = reconstruct_point_cloud(samples, strength_min=args.strength_min)
    save_ply_xyz(args.out, cloud)

    print(f"Captured samples: {len(samples)}")
    print(f"Point cloud points: {len(cloud)}")
    print(f"CSV: {Path(args.csv).resolve()}")
    print(f"PLY: {Path(args.out).resolve()}")


if __name__ == "__main__":
    main()
