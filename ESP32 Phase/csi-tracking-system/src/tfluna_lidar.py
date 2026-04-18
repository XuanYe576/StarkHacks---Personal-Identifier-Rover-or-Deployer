"""
TF-Luna LiDAR UART parsing and point-cloud reconstruction helpers.

TF-Luna frame format (9 bytes):
  0: 0x59
  1: 0x59
  2: Distance low byte (cm)
  3: Distance high byte (cm)
  4: Strength low byte
  5: Strength high byte
  6: Temperature low byte
  7: Temperature high byte
  8: Checksum = sum(bytes[0:8]) & 0xFF
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import struct
import time
from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np
import serial


@dataclass
class TFLunaSample:
    ts: float
    distance_m: float
    strength: int
    temp_c: float
    azimuth_deg: float
    elevation_deg: float


class TFLunaUART:
    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 0.2) -> None:
        self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)

    def close(self) -> None:
        if self.ser and self.ser.is_open:
            self.ser.close()

    def _read_exact(self, n: int) -> Optional[bytes]:
        buf = self.ser.read(n)
        if len(buf) != n:
            return None
        return buf

    def read_sample(self) -> Optional[Tuple[float, int, float]]:
        """
        Returns:
          (distance_m, strength, temp_c) if a valid frame is decoded, else None.
        """
        while True:
            b = self._read_exact(1)
            if b is None:
                return None
            if b[0] != 0x59:
                continue
            b2 = self._read_exact(1)
            if b2 is None:
                return None
            if b2[0] != 0x59:
                continue

            rest = self._read_exact(7)
            if rest is None:
                return None

            frame = bytes([0x59, 0x59]) + rest
            checksum = sum(frame[:8]) & 0xFF
            if checksum != frame[8]:
                continue

            dist_cm = frame[2] | (frame[3] << 8)
            strength = frame[4] | (frame[5] << 8)
            temp_raw = frame[6] | (frame[7] << 8)
            temp_c = (temp_raw / 8.0) - 256.0
            return dist_cm / 100.0, strength, temp_c


def spherical_to_xyz(distance_m: float, azimuth_deg: float, elevation_deg: float) -> Tuple[float, float, float]:
    az = math.radians(azimuth_deg)
    el = math.radians(elevation_deg)
    x = distance_m * math.cos(el) * math.cos(az)
    y = distance_m * math.cos(el) * math.sin(az)
    z = distance_m * math.sin(el)
    return x, y, z


def reconstruct_point_cloud(samples: Sequence[TFLunaSample], strength_min: int = 20) -> np.ndarray:
    """
    Build max-resolution cloud: one point per valid sample (no downsampling).
    """
    pts: List[Tuple[float, float, float]] = []
    for s in samples:
        if s.distance_m <= 0.0 or s.strength < strength_min:
            continue
        pts.append(spherical_to_xyz(s.distance_m, s.azimuth_deg, s.elevation_deg))
    if not pts:
        return np.zeros((0, 3), dtype=np.float32)
    return np.asarray(pts, dtype=np.float32)


def save_ply_xyz(path: str, points_xyz: np.ndarray) -> None:
    n = int(points_xyz.shape[0])
    with open(path, "w", encoding="utf-8") as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {n}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("end_header\n")
        for p in points_xyz:
            f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")


def generate_servo_angles(
    count: int,
    azimuth_min_deg: float = -90.0,
    azimuth_max_deg: float = 90.0,
    elevation_deg: float = 0.0,
) -> List[Tuple[float, float]]:
    """
    Generates a dense sweep from min..max with implicit max angular resolution
    based on sample count.
    """
    if count <= 1:
        return [(azimuth_min_deg, elevation_deg)]
    angles: List[Tuple[float, float]] = []
    span = azimuth_max_deg - azimuth_min_deg
    for i in range(count):
        t = i / float(count - 1)
        angles.append((azimuth_min_deg + t * span, elevation_deg))
    return angles


def capture_sweep(
    port: str,
    baudrate: int,
    seconds: float,
    azimuth_min_deg: float = -90.0,
    azimuth_max_deg: float = 90.0,
    elevation_deg: float = 0.0,
) -> List[TFLunaSample]:
    """
    Capture TF-Luna stream and assign sweep angles across the capture window.
    For best accuracy, feed real angles from your servo controller.
    """
    dev = TFLunaUART(port=port, baudrate=baudrate, timeout=0.2)
    raw: List[Tuple[float, float, int, float]] = []
    t0 = time.time()
    try:
        while time.time() - t0 < seconds:
            out = dev.read_sample()
            if out is None:
                continue
            dist_m, strength, temp_c = out
            raw.append((time.time(), dist_m, strength, temp_c))
    finally:
        dev.close()

    angles = generate_servo_angles(
        len(raw),
        azimuth_min_deg=azimuth_min_deg,
        azimuth_max_deg=azimuth_max_deg,
        elevation_deg=elevation_deg,
    )
    samples: List[TFLunaSample] = []
    for (ts, dist_m, strength, temp_c), (az, el) in zip(raw, angles):
        samples.append(
            TFLunaSample(
                ts=ts,
                distance_m=dist_m,
                strength=strength,
                temp_c=temp_c,
                azimuth_deg=az,
                elevation_deg=el,
            )
        )
    return samples
