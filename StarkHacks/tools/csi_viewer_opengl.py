#!/usr/bin/env python3
import argparse
import json
from pathlib import Path
import re
import sys
import time

import cv2
import numpy as np
import serial

CANVAS_W = 1280
CANVAS_H = 720
PANEL_W = CANVAS_W // 2
PANEL_H = CANVAS_H
MIKU_BLUE = (187, 197, 57)  # BGR for #39C5BB

REPO_ROOT = Path(__file__).resolve().parents[2]
YOLO_REPO = REPO_ROOT / "lib" / "YOLOv7-DeepSORT-Human-Tracking"


def camera_backends():
    if sys.platform == "darwin":
        return [cv2.CAP_AVFOUNDATION, cv2.CAP_ANY]
    if sys.platform.startswith("win"):
        return [cv2.CAP_DSHOW, cv2.CAP_MSMF, cv2.CAP_ANY]
    return [cv2.CAP_ANY]


def parse_args():
    parser = argparse.ArgumentParser(description="ESP32 CSI viewer (OpenCV OpenGL window)")
    parser.add_argument("--port", required=True, help="Serial port (for example /dev/ttyACM0 or COM5)")
    parser.add_argument("--baud", type=int, default=921600, help="Serial baud rate")
    parser.add_argument("--port-b", default="", help="Optional second CSI serial port")
    parser.add_argument("--port-c", default="", help="Optional third CSI serial port")
    parser.add_argument("--baud-b", type=int, default=921600, help="Second CSI serial baud")
    parser.add_argument("--baud-c", type=int, default=921600, help="Third CSI serial baud")
    parser.add_argument("--bins", type=int, default=64, help="Number of CSI bins to display")
    parser.add_argument("--webcam", default="", help="Optional webcam index or path")
    parser.add_argument("--stereo-right", default="", help="Optional right camera index/path for stereo interface")
    parser.add_argument("--stereo-detect", action="store_true", help="Enable detect/parallax overlay")
    parser.add_argument(
        "--detector-backend",
        default="yolo_deepsort",
        choices=["yolo_deepsort", "hog"],
        help="Detection backend for stereo tracking",
    )
    parser.add_argument("--stereo-detect-interval", type=int, default=6, help="Run detector every N frames")
    parser.add_argument("--stereo-max-missed", type=int, default=20, help="Keep last tracked box for up to N missed detects")
    parser.add_argument("--stereo-baseline-m", type=float, default=0.20, help="Stereo baseline in meters")
    parser.add_argument("--stereo-focal-px", type=float, default=0.0, help="Calibrated focal length in pixels")
    parser.add_argument("--stereo-hfov-deg", type=float, default=78.0, help="HFOV used when focal-px is 0")
    parser.add_argument(
        "--yolo-weights",
        default=str(YOLO_REPO / "checkpoints" / "yolov7x.pt"),
        help="YOLOv7 checkpoint path",
    )
    parser.add_argument(
        "--reid-weights",
        default=str(YOLO_REPO / "checkpoints" / "ReID.pb"),
        help="DeepSORT ReID model path",
    )
    parser.add_argument("--camera-serial-port", default="", help="Optional serial plugin port for camera metadata")
    parser.add_argument("--camera-serial-baud", type=int, default=115200, help="Baud for camera serial plugin")
    parser.add_argument("--imu-serial-port", default="", help="Optional IMU serial port (ESP32 IMU stream)")
    parser.add_argument("--imu-serial-baud", type=int, default=115200, help="Baud for IMU serial")
    parser.add_argument("--imu-yaw-offset-deg", type=float, default=0.0, help="Yaw offset to align IMU and camera frame")
    parser.add_argument("--csi-gaussian-overlay", action="store_true", help="Enable Gaussian CSI strength heat overlay")
    parser.add_argument("--csi-gaussian-alpha", type=float, default=0.35, help="Gaussian overlay alpha (0.0 to 1.0)")
    parser.add_argument("--csi-gaussian-sigma", type=float, default=20.0, help="Gaussian spread in pixels")
    parser.add_argument("--camera-fusion-generate", action="store_true", help="Generate CSI-camera fused heat layer")
    parser.add_argument("--camera-fusion-alpha", type=float, default=0.42, help="CSI-camera fusion alpha (0.0 to 1.0)")
    parser.add_argument("--antenna-pov-overlay", action="store_true", help="Enable camera-space Gaussian overlay from antenna POV")
    parser.add_argument("--antenna-pov-alpha", type=float, default=0.35, help="Antenna POV overlay alpha (0.0 to 1.0)")
    parser.add_argument("--antenna-center-x", type=float, default=0.75, help="Antenna origin x in normalized screen coordinates (0..1)")
    parser.add_argument("--antenna-center-y", type=float, default=0.90, help="Antenna origin y in normalized screen coordinates (0..1)")
    parser.add_argument("--antenna-azimuth-deg", type=float, default=-90.0, help="Main-lobe azimuth in degrees (0:right, 90:down, -90:up)")
    parser.add_argument("--antenna-sigma-forward", type=float, default=320.0, help="Forward spread in pixels")
    parser.add_argument("--antenna-sigma-lateral", type=float, default=120.0, help="Lateral spread in pixels")
    parser.add_argument("--antenna-front-softness", type=float, default=80.0, help="Front-only gating softness in pixels")
    parser.add_argument("--presence-threshold", type=float, default=0.25, help="CSI strength threshold for presence color switch")
    parser.add_argument("--fusion-enable", action="store_true", help="Enable WiFi+YOLO+IMU association")
    parser.add_argument("--wifi-look-yaw-deg", type=float, default=0.0, help="Base WiFi look yaw (deg) if no servo/IMU source")
    parser.add_argument("--fusion-assoc-gate-deg", type=float, default=18.0, help="Association gate in degrees")
    parser.add_argument("--fusion-csi-min", type=float, default=0.20, help="Minimum CSI strength for association")
    parser.add_argument("--trilateration-enable", action="store_true", help="Enable multi-ESP RSSI trilateration")
    parser.add_argument("--anchor-a", default="0.0,0.0", help="Anchor A x,y in meters")
    parser.add_argument("--anchor-b", default="1.0,0.0", help="Anchor B x,y in meters")
    parser.add_argument("--anchor-c", default="0.0,1.0", help="Anchor C x,y in meters")
    parser.add_argument("--rssi-ref-db", type=float, default=-40.0, help="RSSI at 1m for distance estimate")
    parser.add_argument("--path-loss-exp", type=float, default=2.2, help="Path-loss exponent for distance estimate")
    return parser.parse_args()


def open_camera(source):
    if source == "":
        return None
    try:
        source = int(source)
    except ValueError:
        pass
    if isinstance(source, int):
        for backend in camera_backends():
            cam = cv2.VideoCapture(source, backend)
            if cam.isOpened():
                return cam
            cam.release()
        return None

    cam = cv2.VideoCapture(source, cv2.CAP_ANY)
    if not cam.isOpened():
        return None
    return cam


def parse_csi_line(line):
    line = line.strip()
    if not line:
        return None

    parts = line.split(",")
    if len(parts) < 6:
        return None

    if parts[0] == "CSIv1":
        try:
            seq = int(parts[1])
            rssi = int(parts[2])
            raw_len = int(parts[3])
            bins = int(parts[4])
        except ValueError:
            return None

        if bins <= 0:
            return None

        expected_len = 7 + (2 * bins)
        if len(parts) != expected_len:
            return None
        if parts[5] != "A":
            return None
        phase_marker_index = 6 + bins
        if parts[phase_marker_index] != "P":
            return None

        try:
            amps = [int(v) for v in parts[6:phase_marker_index]]
            phases = [int(v) for v in parts[phase_marker_index + 1 :]]
        except ValueError:
            return None
        if len(amps) != bins or len(phases) != bins:
            return None
        return {
            "seq": seq,
            "rssi": rssi,
            "raw_len": raw_len,
            "bins": bins,
            "amp": amps,
            "phase": phases,
            "version": "CSIv1",
        }

    # Backward-compatible parser for legacy firmware:
    # CSI,<seq>,<rssi>,<raw_len>,<bins>,<mag_0>,...,<mag_n>
    if parts[0] == "CSI":
        try:
            seq = int(parts[1])
            rssi = int(parts[2])
            raw_len = int(parts[3])
            bins = int(parts[4])
            mags = [int(v) for v in parts[5:]]
        except ValueError:
            return None
        if bins <= 0 or len(mags) == 0:
            return None
        return {
            "seq": seq,
            "rssi": rssi,
            "raw_len": raw_len,
            "bins": bins,
            "amp": mags,
            "phase": [0] * len(mags),
            "version": "CSI",
        }

    return None


def parse_imu_line(line):
    line = line.strip()
    if not line:
        return None

    # CSV formats:
    # IMU,yaw,pitch,roll
    # YPR,yaw,pitch,roll
    parts = line.split(",")
    if len(parts) >= 4 and parts[0].upper() in ("IMU", "YPR"):
        try:
            yaw = float(parts[1])
            pitch = float(parts[2])
            roll = float(parts[3])
            return {"yaw": yaw, "pitch": pitch, "roll": roll}
        except ValueError:
            pass

    # JSON format: {"yaw":..,"pitch":..,"roll":..}
    if line.startswith("{") and line.endswith("}"):
        try:
            obj = json.loads(line)
            if "yaw" in obj:
                return {
                    "yaw": float(obj.get("yaw", 0.0)),
                    "pitch": float(obj.get("pitch", 0.0)),
                    "roll": float(obj.get("roll", 0.0)),
                }
        except Exception:
            pass

    # Key-value format: yaw=.. pitch=.. roll=..
    m_yaw = re.search(r"yaw\s*=\s*(-?\d+(\.\d+)?)", line, flags=re.IGNORECASE)
    if m_yaw:
        m_pitch = re.search(r"pitch\s*=\s*(-?\d+(\.\d+)?)", line, flags=re.IGNORECASE)
        m_roll = re.search(r"roll\s*=\s*(-?\d+(\.\d+)?)", line, flags=re.IGNORECASE)
        return {
            "yaw": float(m_yaw.group(1)),
            "pitch": float(m_pitch.group(1)) if m_pitch else 0.0,
            "roll": float(m_roll.group(1)) if m_roll else 0.0,
        }
    return None


def wrap_deg(x):
    return ((x + 180.0) % 360.0) - 180.0


def angle_diff_deg(a, b):
    return abs(wrap_deg(a - b))


def bbox_yaw_deg(bbox, frame_w, hfov_deg):
    if bbox is None or frame_w <= 0:
        return None
    x, _, w, _ = bbox
    cx = x + 0.5 * w
    nx = (cx / frame_w) - 0.5
    return float(nx * hfov_deg)


def parse_anchor_xy(s, fallback=(0.0, 0.0)):
    try:
        p = [float(x.strip()) for x in s.split(",")]
        if len(p) == 2:
            return (p[0], p[1])
    except Exception:
        pass
    return fallback


def rssi_to_distance_m(rssi_dbm, rssi_ref_db, path_loss_exp):
    n = max(1e-3, float(path_loss_exp))
    # Log-distance path loss model:
    # d = 10 ^ ((RSSI_ref(1m) - RSSI) / (10*n))
    d = 10.0 ** ((float(rssi_ref_db) - float(rssi_dbm)) / (10.0 * n))
    return float(np.clip(d, 0.1, 50.0))


def trilaterate_2d(a, b, c, da, db, dc):
    # Linearized least-squares from three circles.
    x1, y1 = a
    x2, y2 = b
    x3, y3 = c
    A = np.array([
        [2.0 * (x2 - x1), 2.0 * (y2 - y1)],
        [2.0 * (x3 - x1), 2.0 * (y3 - y1)],
    ], dtype=np.float64)
    B = np.array([
        (x2 * x2 + y2 * y2 - db * db) - (x1 * x1 + y1 * y1 - da * da),
        (x3 * x3 + y3 * y3 - dc * dc) - (x1 * x1 + y1 * y1 - da * da),
    ], dtype=np.float64)
    try:
        pos, *_ = np.linalg.lstsq(A, B, rcond=None)
        return float(pos[0]), float(pos[1])
    except Exception:
        return None


def draw_overlay(img, text, y, color=(255, 255, 255)):
    cv2.putText(img, text, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA)
    cv2.putText(img, text, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (20, 20, 20), 1, cv2.LINE_AA)


def draw_status_box(img, lines, x=10, y=10, line_h=22, text_color=(255, 255, 255), border_color=(60, 60, 60)):
    if not lines:
        return
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.55
    thickness = 1
    max_w = 0
    for ln in lines:
        (w, _), _ = cv2.getTextSize(ln, font, scale, thickness)
        max_w = max(max_w, w)
    box_w = max_w + 16
    box_h = line_h * len(lines) + 10
    x2 = min(img.shape[1] - 1, x + box_w)
    y2 = min(img.shape[0] - 1, y + box_h)
    cv2.rectangle(img, (x, y), (x2, y2), (0, 0, 0), -1)
    cv2.rectangle(img, (x, y), (x2, y2), border_color, 1)
    for i, ln in enumerate(lines):
        yy = y + 20 + i * line_h
        cv2.putText(img, ln, (x + 8, yy), font, scale, text_color, 2, cv2.LINE_AA)
        cv2.putText(img, ln, (x + 8, yy), font, scale, (20, 20, 20), 1, cv2.LINE_AA)


def draw_wave(amp_values, width=900, height=680, color=(0, 0, 255)):
    canvas = np.zeros((height, width, 3), dtype=np.uint8)
    if amp_values is None or len(amp_values) == 0:
        return canvas

    amp = np.array(amp_values, dtype=np.float32)
    amp = np.clip(amp, 0.0, 255.0)
    vmax = max(32.0, float(np.percentile(amp, 98)))
    amp = np.clip(amp / vmax, 0.0, 1.0)

    n = amp.shape[0]
    if n == 1:
        x = np.array([width // 2], dtype=np.int32)
    else:
        x = np.linspace(0, width - 1, n).astype(np.int32)
    y = (height - 20 - (amp * (height - 40))).astype(np.int32)

    pts = np.stack([x, y], axis=1).reshape((-1, 1, 2))
    cv2.polylines(canvas, [pts], isClosed=False, color=color, thickness=2)
    return canvas


def draw_wave_on_canvas(canvas, amp_values, color=(0, 0, 255), inset=20):
    if amp_values is None or len(amp_values) == 0:
        return
    h, w = canvas.shape[:2]
    amp = np.array(amp_values, dtype=np.float32)
    amp = np.clip(amp, 0.0, 255.0)
    vmax = max(32.0, float(np.percentile(amp, 98)))
    amp = np.clip(amp / vmax, 0.0, 1.0)
    n = amp.shape[0]
    if n == 1:
        x = np.array([w // 2], dtype=np.int32)
    else:
        x = np.linspace(0, w - 1, n).astype(np.int32)
    y = (h - inset - (amp * (h - (2 * inset)))).astype(np.int32)
    pts = np.stack([x, y], axis=1).reshape((-1, 1, 2))
    cv2.polylines(canvas, [pts], isClosed=False, color=color, thickness=2)


def build_gaussian_strength_overlay(amp_values, width, height, inset=20, sigma=20.0):
    overlay = np.zeros((height, width, 3), dtype=np.uint8)
    if amp_values is None or len(amp_values) == 0:
        return overlay

    amp = np.array(amp_values, dtype=np.float32)
    amp = np.clip(amp, 0.0, 255.0)
    vmax = max(32.0, float(np.percentile(amp, 98)))
    norm = np.clip(amp / vmax, 0.0, 1.0)
    if norm.size == 0:
        return overlay

    n = norm.shape[0]
    if n == 1:
        x = np.array([width // 2], dtype=np.int32)
    else:
        x = np.linspace(0, width - 1, n).astype(np.int32)
    y = (height - inset - (norm * (height - (2 * inset)))).astype(np.int32)
    y = np.clip(y, 0, height - 1)

    heat = np.zeros((height, width), dtype=np.float32)
    for xi, yi, vi in zip(x, y, norm):
        if vi <= 0.0:
            continue
        heat[yi, xi] = max(heat[yi, xi], vi)

    sigma = max(1.0, float(sigma))
    heat = cv2.GaussianBlur(heat, (0, 0), sigmaX=sigma, sigmaY=sigma)
    peak = float(np.max(heat))
    if peak > 1e-6:
        heat = heat / peak
    heat_u8 = (heat * 255.0).astype(np.uint8)
    overlay[:, :, 2] = heat_u8
    overlay[:, :, 1] = (heat_u8 * 0.25).astype(np.uint8)
    return overlay


def draw_csi_layers(canvas, latest_amp, fft_vals, args, inset):
    if args.csi_gaussian_overlay:
        alpha = float(np.clip(args.csi_gaussian_alpha, 0.0, 1.0))
        gauss = build_gaussian_strength_overlay(
            latest_amp,
            canvas.shape[1],
            canvas.shape[0],
            inset=inset,
            sigma=args.csi_gaussian_sigma,
        )
        cv2.addWeighted(canvas, 1.0, gauss, alpha, 0.0, dst=canvas)
    draw_wave_on_canvas(canvas, fft_vals, color=(255, 255, 0), inset=inset)
    draw_wave_on_canvas(canvas, latest_amp, color=(0, 0, 255), inset=inset)


def draw_camera_fusion_layer(canvas, latest_amp, csi_strength, args, inset=80):
    if not args.camera_fusion_generate:
        return
    alpha = float(np.clip(args.camera_fusion_alpha, 0.0, 1.0))
    if alpha <= 0.0:
        return
    # Two cues fused: per-bin Gaussian field + antenna POV field weighted by CSI strength.
    gauss = build_gaussian_strength_overlay(
        latest_amp,
        canvas.shape[1],
        canvas.shape[0],
        inset=inset,
        sigma=args.csi_gaussian_sigma,
    )
    pov = build_antenna_pov_overlay(csi_strength, canvas.shape[1], canvas.shape[0], args)
    fused = cv2.addWeighted(gauss, 0.55, pov, 0.45, 0.0)
    cv2.addWeighted(canvas, 1.0, fused, alpha, 0.0, dst=canvas)


def compute_csi_strength(amp_values, rssi_dbm):
    if amp_values is None or len(amp_values) == 0:
        return float(np.clip((rssi_dbm + 95.0) / 60.0, 0.0, 1.0))
    amp = np.asarray(amp_values, dtype=np.float32)
    amp = np.clip(amp, 0.0, 255.0)
    if amp.size == 0:
        return float(np.clip((rssi_dbm + 95.0) / 60.0, 0.0, 1.0))
    p95 = max(24.0, float(np.percentile(amp, 95)))
    level = float(np.mean(amp) / p95)
    rssi_level = float(np.clip((rssi_dbm + 95.0) / 60.0, 0.0, 1.0))
    return float(np.clip(0.8 * level + 0.2 * rssi_level, 0.0, 1.0))


def build_antenna_pov_overlay(strength, width, height, args):
    overlay = np.zeros((height, width, 3), dtype=np.uint8)
    strength = float(np.clip(strength, 0.0, 1.0))
    if strength <= 1e-4:
        return overlay

    cx = float(np.clip(args.antenna_center_x, 0.0, 1.0)) * (width - 1)
    cy = float(np.clip(args.antenna_center_y, 0.0, 1.0)) * (height - 1)
    theta = np.radians(float(args.antenna_azimuth_deg))
    c = float(np.cos(theta))
    s = float(np.sin(theta))

    xs = np.arange(width, dtype=np.float32)
    ys = np.arange(height, dtype=np.float32)
    xx, yy = np.meshgrid(xs, ys)
    dx = xx - cx
    dy = yy - cy

    # Rotate into antenna frame: u=forward axis, v=lateral axis.
    u = (dx * c) + (dy * s)
    v = (-dx * s) + (dy * c)

    sigma_u = max(8.0, float(args.antenna_sigma_forward))
    sigma_v = max(8.0, float(args.antenna_sigma_lateral))
    front_soft = max(1.0, float(args.antenna_front_softness))

    gauss = np.exp(-0.5 * ((u / sigma_u) ** 2 + (v / sigma_v) ** 2))
    front_gate = 1.0 / (1.0 + np.exp(-u / front_soft))
    heat = gauss * front_gate * strength * 2.0
    heat = np.clip(heat, 0.0, 1.0)

    heat_u8 = (heat * 255.0).astype(np.uint8)
    overlay[:, :, 2] = heat_u8
    overlay[:, :, 1] = (heat_u8 * 0.15).astype(np.uint8)
    return overlay


def draw_antenna_pov_overlay(canvas, strength, args):
    if not args.antenna_pov_overlay:
        return
    alpha = float(np.clip(args.antenna_pov_alpha, 0.0, 1.0))
    if alpha <= 0.0:
        return
    ov = build_antenna_pov_overlay(strength, canvas.shape[1], canvas.shape[0], args)
    cv2.addWeighted(canvas, 1.0, ov, alpha, 0.0, dst=canvas)


def fft_magnitude(values, out_bins):
    arr = np.asarray(values, dtype=np.float32)
    if arr.size == 0:
        return np.zeros((out_bins,), dtype=np.float32)
    arr = arr - np.mean(arr)
    spec = np.fft.rfft(arr)
    mag = np.abs(spec)
    if mag.size > out_bins:
        mag = mag[:out_bins]
    out = np.zeros((out_bins,), dtype=np.float32)
    out[: mag.shape[0]] = mag
    return out


def detect_primary_person(frame, hog):
    small = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
    rects, weights = hog.detectMultiScale(small, winStride=(8, 8), padding=(8, 8), scale=1.05)
    if len(rects) == 0:
        return None
    best_idx = int(np.argmax(weights)) if len(weights) else 0
    x, y, w, h = rects[best_idx]
    x *= 2
    y *= 2
    w *= 2
    h *= 2
    return (int(x), int(y), int(w), int(h))


def detect_primary_person_yolo_deepsort(frame_bgr, detector, tracker):
    frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    detections = detector.detect(frame_rgb)
    if detections is None or len(detections) == 0:
        bboxes = np.empty((0, 4), dtype=np.float32)
        scores = np.empty((0,), dtype=np.float32)
    else:
        bboxes = detections[:, :4].copy()
        bboxes[:, 2:] = bboxes[:, 2:] - bboxes[:, :2]
        scores = detections[:, 4].astype(np.float32)
    tracker.track(frame_rgb, bboxes, scores)

    best = None
    best_area = -1.0
    for trk in tracker.tracker.tracks:
        if not trk.is_confirmed() or trk.time_since_update > 1:
            continue
        x1, y1, x2, y2 = trk.to_tlbr().astype(np.int32)
        w = max(0, x2 - x1)
        h = max(0, y2 - y1)
        area = float(w * h)
        if area > best_area:
            best_area = area
            best = (int(x1), int(y1), int(w), int(h), int(trk.track_id))
    return best


def render_stereo_panel(frame_left, frame_right, info_line):
    left = fit_frame_with_padding(frame_left, PANEL_W, PANEL_H)
    right = fit_frame_with_padding(frame_right, PANEL_W, PANEL_H)
    panel = np.zeros((CANVAS_H, CANVAS_W, 3), dtype=np.uint8)
    # Requested order: [cam1][cam0]
    panel[:, 0:PANEL_W, :] = right
    panel[:, PANEL_W:CANVAS_W, :] = left
    cv2.line(panel, (PANEL_W, 0), (PANEL_W, CANVAS_H - 1), (50, 50, 50), 1)
    draw_overlay(panel, "CAM1", 24)
    cv2.putText(panel, "CAM0", (PANEL_W + 12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(panel, "CAM0", (PANEL_W + 12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (20, 20, 20), 1, cv2.LINE_AA)
    cv2.putText(panel, info_line, (12, CANVAS_H - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(panel, info_line, (12, CANVAS_H - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (20, 20, 20), 1, cv2.LINE_AA)
    return panel


def fit_frame_with_padding(frame, target_w, target_h):
    h, w = frame.shape[:2]
    if h <= 0 or w <= 0:
        return np.zeros((target_h, target_w, 3), dtype=np.uint8)
    scale = min(target_w / float(w), target_h / float(h))
    new_w = max(1, int(round(w * scale)))
    new_h = max(1, int(round(h * scale)))
    resized = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_AREA)
    out = np.zeros((target_h, target_w, 3), dtype=np.uint8)
    x0 = (target_w - new_w) // 2
    y0 = (target_h - new_h) // 2
    out[y0:y0 + new_h, x0:x0 + new_w] = resized
    return out


def main():
    args = parse_args()
    args.bins = max(8, args.bins)
    args.stereo_detect_interval = max(1, args.stereo_detect_interval)
    args.stereo_max_missed = max(1, args.stereo_max_missed)

    def _open_serial_or_warn(port, baud, timeout=0.03, label="serial"):
        if not port:
            return None
        try:
            return serial.Serial(port, baud, timeout=timeout)
        except Exception as exc:
            print(f"[WARN] {label} disabled: {exc}")
            return None

    ser = _open_serial_or_warn(args.port, args.baud, timeout=0.03, label="CSI A serial")
    if ser is None:
        raise RuntimeError("Primary CSI serial port is required and failed to open.")
    ser_b = _open_serial_or_warn(args.port_b, args.baud_b, timeout=0.0, label="CSI B serial")
    ser_c = _open_serial_or_warn(args.port_c, args.baud_c, timeout=0.0, label="CSI C serial")

    anchor_a = parse_anchor_xy(args.anchor_a, fallback=(0.0, 0.0))
    anchor_b = parse_anchor_xy(args.anchor_b, fallback=(1.0, 0.0))
    anchor_c = parse_anchor_xy(args.anchor_c, fallback=(0.0, 1.0))
    cam_left = open_camera(args.webcam)
    cam_right = open_camera(args.stereo_right) if args.stereo_right else None
    hog = None
    yolo_detector = None
    deepsort_left = None
    deepsort_right = None
    if args.stereo_detect:
        if args.detector_backend == "hog":
            hog = cv2.HOGDescriptor()
            hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        else:
            try:
                if str(YOLO_REPO) not in sys.path:
                    sys.path.insert(0, str(YOLO_REPO))
                from api.yolo import YOLOPersonDetector
                from api.deepsort import DeepSORTTracker
                yolo_detector = YOLOPersonDetector()
                yolo_detector.load(args.yolo_weights)
                deepsort_left = DeepSORTTracker(args.reid_weights, 0.5, 30)
                deepsort_right = DeepSORTTracker(args.reid_weights, 0.5, 30)
            except Exception as exc:
                raise RuntimeError(
                    f"Failed to init YOLO+DeepSORT backend: {exc}\n"
                    "Use the YOLO venv and ensure weights exist in lib/YOLOv7-DeepSORT-Human-Tracking/checkpoints/"
                ) from exc
    camera_serial = None
    if args.camera_serial_port:
        try:
            camera_serial = serial.Serial(args.camera_serial_port, args.camera_serial_baud, timeout=0.0)
        except Exception as exc:
            print(f"[WARN] camera serial plugin disabled: {exc}")
            camera_serial = None
    imu_serial = None
    if args.imu_serial_port:
        try:
            imu_serial = serial.Serial(args.imu_serial_port, args.imu_serial_baud, timeout=0.0)
        except Exception as exc:
            print(f"[WARN] IMU serial disabled: {exc}")
            imu_serial = None

    latest_seq = 0
    latest_rssi = -127
    latest_len = 0
    latest_version = "N/A"
    latest_amp = np.zeros((args.bins,), dtype=np.uint8)
    latest_cam_serial = "N/A"
    fps_counter = 0
    fps_time = time.time()
    fps = 0.0
    stereo_info = "stereo off"
    disparity_text = "d=N/A"
    distance_text = "z=N/A"
    detect_frame_i = 0
    cached_left_box = None
    cached_right_box = None
    cached_left_id = None
    cached_right_id = None
    missed_left = 0
    missed_right = 0
    csi_strength_smoothed = 0.0
    csi_strength_smoothed_b = 0.0
    csi_strength_smoothed_c = 0.0
    latest_rssi_b = -127
    latest_rssi_c = -127
    seen_b = False
    seen_c = False
    trilat_xy = None
    trilat_d = (0.0, 0.0, 0.0)
    latest_imu = {"yaw": 0.0, "pitch": 0.0, "roll": 0.0}
    fused_assoc = False
    fused_assoc_err = 999.0
    fused_wifi_yaw = 0.0
    fused_yolo_yaw = 0.0

    window_name = "CSI OpenGL Viewer"
    try:
        cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE | cv2.WINDOW_OPENGL)
    except cv2.error:
        cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)

    try:
        while True:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            parsed = parse_csi_line(line) if line else None
            if parsed is not None:
                latest_seq = parsed["seq"]
                latest_rssi = parsed["rssi"]
                latest_len = parsed["raw_len"]
                latest_version = parsed["version"]
                amp = np.array(parsed["amp"][: args.bins], dtype=np.float32)
                latest_amp[:] = 0
                if amp.size > 0:
                    amp = np.clip(amp, 0.0, 255.0)
                    latest_amp[: amp.shape[0]] = amp.astype(np.uint8)

            if ser_b is not None:
                line_b = ser_b.readline().decode("utf-8", errors="ignore").strip()
                parsed_b = parse_csi_line(line_b) if line_b else None
                if parsed_b is not None:
                    latest_rssi_b = parsed_b["rssi"]
                    seen_b = True
            if ser_c is not None:
                line_c = ser_c.readline().decode("utf-8", errors="ignore").strip()
                parsed_c = parse_csi_line(line_c) if line_c else None
                if parsed_c is not None:
                    latest_rssi_c = parsed_c["rssi"]
                    seen_c = True

            if camera_serial is not None:
                cam_line = camera_serial.readline().decode("utf-8", errors="ignore").strip()
                if cam_line:
                    latest_cam_serial = cam_line
            if imu_serial is not None:
                imu_line = imu_serial.readline().decode("utf-8", errors="ignore").strip()
                parsed_imu = parse_imu_line(imu_line) if imu_line else None
                if parsed_imu is not None:
                    latest_imu = parsed_imu

            fft_vals = fft_magnitude(latest_amp, args.bins)
            csi_strength_raw = compute_csi_strength(latest_amp, latest_rssi)
            csi_strength_smoothed = (0.85 * csi_strength_smoothed) + (0.15 * csi_strength_raw)
            if seen_b:
                csi_strength_smoothed_b = (0.85 * csi_strength_smoothed_b) + (0.15 * float(np.clip((latest_rssi_b + 95.0) / 60.0, 0.0, 1.0)))
            if seen_c:
                csi_strength_smoothed_c = (0.85 * csi_strength_smoothed_c) + (0.15 * float(np.clip((latest_rssi_c + 95.0) / 60.0, 0.0, 1.0)))

            csi_max_strength = max(csi_strength_smoothed, csi_strength_smoothed_b, csi_strength_smoothed_c)
            wifi_present = csi_max_strength >= float(args.presence_threshold)

            if args.trilateration_enable and seen_b and seen_c:
                d_a = rssi_to_distance_m(latest_rssi, args.rssi_ref_db, args.path_loss_exp)
                d_b = rssi_to_distance_m(latest_rssi_b, args.rssi_ref_db, args.path_loss_exp)
                d_c = rssi_to_distance_m(latest_rssi_c, args.rssi_ref_db, args.path_loss_exp)
                trilat_d = (d_a, d_b, d_c)
                trilat_xy = trilaterate_2d(anchor_a, anchor_b, anchor_c, d_a, d_b, d_c)
            wave = np.zeros((CANVAS_H, CANVAS_W, 3), dtype=np.uint8)

            if cam_left is not None and cam_right is not None:
                ok_l, frame_l = cam_left.read()
                ok_r, frame_r = cam_right.read()
                if ok_l and ok_r:
                    detect_frame_i += 1
                    if detect_frame_i % args.stereo_detect_interval == 0:
                        if args.detector_backend == "hog":
                            det_left = detect_primary_person(frame_l, hog)
                            det_right = detect_primary_person(frame_r, hog)
                            if det_left is not None:
                                det_left = (*det_left, -1)
                            if det_right is not None:
                                det_right = (*det_right, -1)
                        else:
                            det_left = detect_primary_person_yolo_deepsort(frame_l, yolo_detector, deepsort_left)
                            det_right = detect_primary_person_yolo_deepsort(frame_r, yolo_detector, deepsort_right)

                        if det_left is not None:
                            cached_left_box = det_left[:4]
                            cached_left_id = det_left[4]
                            missed_left = 0
                        else:
                            missed_left += 1
                            if missed_left > args.stereo_max_missed:
                                cached_left_box = None
                                cached_left_id = None

                        if det_right is not None:
                            cached_right_box = det_right[:4]
                            cached_right_id = det_right[4]
                            missed_right = 0
                        else:
                            missed_right += 1
                            if missed_right > args.stereo_max_missed:
                                cached_right_box = None
                                cached_right_id = None

                    human_present = (cached_left_box is not None) or (cached_right_box is not None)
                    presence_on = wifi_present or human_present
                    accent_color = MIKU_BLUE if presence_on else (255, 255, 255)
                    box_color = MIKU_BLUE if presence_on else (0, 255, 255)

                    if cached_left_box is not None:
                        x, y, w, h = cached_left_box
                        cv2.rectangle(frame_l, (x, y), (x + w, y + h), box_color, 2)
                        if cached_left_id is not None and cached_left_id >= 0:
                            cv2.putText(frame_l, f"ID {cached_left_id}", (x, max(20, y - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, box_color, 2, cv2.LINE_AA)
                    if cached_right_box is not None:
                        x, y, w, h = cached_right_box
                        cv2.rectangle(frame_r, (x, y), (x + w, y + h), box_color, 2)
                        if cached_right_id is not None and cached_right_id >= 0:
                            cv2.putText(frame_r, f"ID {cached_right_id}", (x, max(20, y - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, box_color, 2, cv2.LINE_AA)

                    stereo_info = f"stereo {args.detector_backend}"
                    if cached_left_box is not None and cached_right_box is not None:
                        cx_l = cached_left_box[0] + (cached_left_box[2] * 0.5)
                        cx_r = cached_right_box[0] + (cached_right_box[2] * 0.5)
                        disparity = cx_l - cx_r
                        disparity_text = f"d={disparity:.1f}px"
                        if disparity > 1.0:
                            focal_px = args.stereo_focal_px
                            if focal_px <= 0.0:
                                focal_px = (frame_l.shape[1] * 0.5) / np.tan(np.radians(args.stereo_hfov_deg * 0.5))
                            depth = (args.stereo_baseline_m * focal_px) / disparity
                            distance_text = f"z={depth:.2f}m"
                            stereo_info = f"stereo detect ON"
                        else:
                            distance_text = "z=N/A"
                            stereo_info = f"stereo detect ON"
                    else:
                        disparity_text = "d=N/A"
                        distance_text = "z=N/A"

                    fused_assoc = False
                    fused_assoc_err = 999.0
                    fused_wifi_yaw = wrap_deg(
                        float(args.wifi_look_yaw_deg)
                        + float(args.imu_yaw_offset_deg)
                        + float(latest_imu["yaw"])
                    )
                    yolo_yaw = bbox_yaw_deg(cached_left_box, frame_l.shape[1], float(args.stereo_hfov_deg))
                    if yolo_yaw is None:
                        yolo_yaw = bbox_yaw_deg(cached_right_box, frame_r.shape[1], float(args.stereo_hfov_deg))
                    if yolo_yaw is not None:
                        fused_yolo_yaw = yolo_yaw
                        fused_assoc_err = angle_diff_deg(fused_wifi_yaw, yolo_yaw)
                        if args.fusion_enable and csi_strength_smoothed >= float(args.fusion_csi_min):
                            fused_assoc = fused_assoc_err <= float(args.fusion_assoc_gate_deg)

                    stereo_panel = render_stereo_panel(frame_l, frame_r, stereo_info[:54])
                    draw_camera_fusion_layer(stereo_panel, latest_amp, csi_strength_smoothed, args, inset=90)
                    draw_antenna_pov_overlay(stereo_panel, csi_strength_smoothed, args)
                    # Overlay CSI graph directly over [cam1][cam0] interface.
                    draw_csi_layers(stereo_panel, latest_amp, fft_vals, args, inset=90)
                    status_lines = [
                        "CSI overlay: RAW=Red FFT=Yellow" + (" + Gaussian" if args.csi_gaussian_overlay else ""),
                        f"seq={latest_seq} rssi={latest_rssi} len={latest_len} fmt={latest_version}",
                        f"csi_strength(a,b,c)=({csi_strength_smoothed:.2f},{csi_strength_smoothed_b:.2f},{csi_strength_smoothed_c:.2f})",
                        f"presence={'ON' if presence_on else 'OFF'}",
                        f"imu(ypr)=({latest_imu['yaw']:.1f},{latest_imu['pitch']:.1f},{latest_imu['roll']:.1f})",
                        f"fusion={'ON' if args.fusion_enable else 'OFF'} assoc={'YES' if fused_assoc else 'NO'} err={fused_assoc_err:.1f}deg",
                        f"{disparity_text}  {distance_text}",
                        f"mode={stereo_info}",
                        f"camera_fusion={'ON' if args.camera_fusion_generate else 'OFF'} alpha={float(args.camera_fusion_alpha):.2f}",
                    ]
                    if args.trilateration_enable:
                        if trilat_xy is not None:
                            status_lines.append(f"trilat_xy=({trilat_xy[0]:.2f},{trilat_xy[1]:.2f})m d=({trilat_d[0]:.2f},{trilat_d[1]:.2f},{trilat_d[2]:.2f})m")
                        else:
                            status_lines.append("trilat_xy=N/A (need 3 CSI nodes: A+B+C)")
                    if camera_serial is not None:
                        status_lines.append(f"serial={latest_cam_serial[:64]}")
                    status_lines.append("Press q to quit")
                    draw_status_box(stereo_panel, status_lines, x=10, y=34, line_h=22, text_color=accent_color, border_color=accent_color)
                    if presence_on:
                        cv2.rectangle(stereo_panel, (0, 0), (stereo_panel.shape[1] - 1, stereo_panel.shape[0] - 1), MIKU_BLUE, 2)
                    if fused_assoc:
                        cv2.putText(stereo_panel, "WIFI<->YOLO ASSOCIATED", (PANEL_W - 145, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.55, MIKU_BLUE, 2, cv2.LINE_AA)
                        cv2.putText(stereo_panel, "WIFI<->YOLO ASSOCIATED", (PANEL_W - 145, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (20, 20, 20), 1, cv2.LINE_AA)
                    canvas = stereo_panel
                else:
                    fallback = np.zeros((CANVAS_H, CANVAS_W, 3), dtype=np.uint8)
                    draw_overlay(fallback, "Stereo camera read failed", 40)
                    draw_csi_layers(fallback, latest_amp, fft_vals, args, inset=80)
                    canvas = fallback
            elif cam_left is not None:
                ok, frame = cam_left.read()
                if ok:
                    frame = fit_frame_with_padding(frame, CANVAS_W, CANVAS_H)
                    if camera_serial is not None:
                        draw_overlay(frame, f"Serial: {latest_cam_serial[:48]}", 40)
                    draw_overlay(frame, "Stereo off (set --stereo-right)", 66)
                else:
                    frame = np.zeros((CANVAS_H, CANVAS_W, 3), dtype=np.uint8)
                    draw_overlay(frame, "Webcam read failed", 40)
                presence_on = wifi_present
                accent_color = MIKU_BLUE if presence_on else (255, 255, 255)
                draw_camera_fusion_layer(frame, latest_amp, csi_strength_smoothed, args, inset=80)
                draw_antenna_pov_overlay(frame, csi_strength_smoothed, args)
                draw_csi_layers(frame, latest_amp, fft_vals, args, inset=80)
                draw_overlay(frame, f"seq={latest_seq} rssi={latest_rssi} len={latest_len} fmt={latest_version}", 92, color=accent_color)
                draw_overlay(frame, f"csi_strength(a,b,c)=({csi_strength_smoothed:.2f},{csi_strength_smoothed_b:.2f},{csi_strength_smoothed_c:.2f}) presence={'ON' if presence_on else 'OFF'}", 118, color=accent_color)
                draw_overlay(frame, f"imu_yaw={latest_imu['yaw']:.1f} fusion={'ON' if args.fusion_enable else 'OFF'}", 144, color=accent_color)
                if args.trilateration_enable and trilat_xy is not None:
                    draw_overlay(frame, f"trilat_xy=({trilat_xy[0]:.2f},{trilat_xy[1]:.2f})m", 170, color=accent_color)
                    draw_overlay(frame, "Press q to quit", 196, color=accent_color)
                else:
                    draw_overlay(frame, "Press q to quit", 170, color=accent_color)
                if presence_on:
                    cv2.rectangle(frame, (0, 0), (frame.shape[1] - 1, frame.shape[0] - 1), MIKU_BLUE, 2)
                canvas = frame
            else:
                presence_on = wifi_present
                accent_color = MIKU_BLUE if presence_on else (255, 255, 255)
                draw_csi_layers(wave, latest_amp, fft_vals, args, inset=20)
                draw_overlay(
                    wave,
                    "Combined CSI Waves: RAW=Red FFT=Yellow" + (" + Gaussian" if args.csi_gaussian_overlay else ""),
                    26,
                    color=accent_color,
                )
                draw_overlay(wave, f"CSI seq: {latest_seq}", 52, color=accent_color)
                draw_overlay(wave, f"RSSI: {latest_rssi} dBm", 78, color=accent_color)
                draw_overlay(wave, f"Raw len: {latest_len}", 104, color=accent_color)
                draw_overlay(wave, f"Format: {latest_version}", 130, color=accent_color)
                draw_overlay(wave, f"RSSI(b,c)=({latest_rssi_b},{latest_rssi_c}) dBm", 156, color=accent_color)
                if args.trilateration_enable and trilat_xy is not None:
                    draw_overlay(wave, f"Trilat XY: ({trilat_xy[0]:.2f},{trilat_xy[1]:.2f})m", 182, color=accent_color)
                    draw_overlay(wave, f"Presence: {'ON' if presence_on else 'OFF'}", 208, color=accent_color)
                    draw_overlay(wave, "Press q to quit", 234, color=accent_color)
                else:
                    draw_overlay(wave, f"Presence: {'ON' if presence_on else 'OFF'}", 182, color=accent_color)
                    draw_overlay(wave, "Press q to quit", 208, color=accent_color)
                if presence_on:
                    cv2.rectangle(wave, (0, 0), (wave.shape[1] - 1, wave.shape[0] - 1), MIKU_BLUE, 2)
                canvas = wave

            now = time.time()
            fps_counter += 1
            if now - fps_time >= 1.0:
                fps = fps_counter / (now - fps_time)
                fps_counter = 0
                fps_time = now
            fps_text = f"FPS {fps:.1f}"
            (fw, fh), _ = cv2.getTextSize(fps_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
            fx = max(8, canvas.shape[1] - fw - 18)
            fy = max(24, canvas.shape[0] - 14)
            cv2.rectangle(canvas, (fx - 8, fy - fh - 8), (fx + fw + 8, fy + 6), (0, 0, 0), -1)
            cv2.putText(canvas, fps_text, (fx, fy), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(canvas, fps_text, (fx, fy), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (20, 20, 20), 1, cv2.LINE_AA)

            cv2.imshow(window_name, canvas)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
    finally:
        ser.close()
        if ser_b is not None:
            ser_b.close()
        if ser_c is not None:
            ser_c.close()
        if camera_serial is not None:
            camera_serial.close()
        if imu_serial is not None:
            imu_serial.close()
        if cam_left is not None:
            cam_left.release()
        if cam_right is not None:
            cam_right.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
