#!/usr/bin/env python3
import argparse
from pathlib import Path
import sys
import time

import cv2
import numpy as np
import serial

CANVAS_W = 1280
CANVAS_H = 720
PANEL_W = CANVAS_W // 2
PANEL_H = CANVAS_H

REPO_ROOT = Path(__file__).resolve().parents[2]
YOLO_REPO = REPO_ROOT / "third_party" / "YOLOv7-DeepSORT-Human-Tracking"


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


def draw_overlay(img, text, y):
    cv2.putText(img, text, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(img, text, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (20, 20, 20), 1, cv2.LINE_AA)


def draw_status_box(img, lines, x=10, y=10, line_h=22):
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
    cv2.rectangle(img, (x, y), (x2, y2), (60, 60, 60), 1)
    for i, ln in enumerate(lines):
        yy = y + 20 + i * line_h
        cv2.putText(img, ln, (x + 8, yy), font, scale, (255, 255, 255), 2, cv2.LINE_AA)
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

    ser = serial.Serial(args.port, args.baud, timeout=0.03)
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
                    "Use the YOLO venv and ensure weights exist in third_party/YOLOv7-DeepSORT-Human-Tracking/checkpoints/"
                ) from exc
    camera_serial = None
    if args.camera_serial_port:
        try:
            camera_serial = serial.Serial(args.camera_serial_port, args.camera_serial_baud, timeout=0.0)
        except Exception as exc:
            print(f"[WARN] camera serial plugin disabled: {exc}")
            camera_serial = None

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

            if camera_serial is not None:
                cam_line = camera_serial.readline().decode("utf-8", errors="ignore").strip()
                if cam_line:
                    latest_cam_serial = cam_line

            fft_vals = fft_magnitude(latest_amp, args.bins)
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

                    if cached_left_box is not None:
                        x, y, w, h = cached_left_box
                        cv2.rectangle(frame_l, (x, y), (x + w, y + h), (0, 255, 255), 2)
                        if cached_left_id is not None and cached_left_id >= 0:
                            cv2.putText(frame_l, f"ID {cached_left_id}", (x, max(20, y - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2, cv2.LINE_AA)
                    if cached_right_box is not None:
                        x, y, w, h = cached_right_box
                        cv2.rectangle(frame_r, (x, y), (x + w, y + h), (0, 255, 255), 2)
                        if cached_right_id is not None and cached_right_id >= 0:
                            cv2.putText(frame_r, f"ID {cached_right_id}", (x, max(20, y - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2, cv2.LINE_AA)

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

                    stereo_panel = render_stereo_panel(frame_l, frame_r, stereo_info[:54])
                    # Overlay CSI graph directly over [cam1][cam0] interface.
                    draw_wave_on_canvas(stereo_panel, fft_vals, color=(255, 255, 0), inset=90)
                    draw_wave_on_canvas(stereo_panel, latest_amp, color=(0, 0, 255), inset=90)
                    status_lines = [
                        "CSI overlay: RAW=Red FFT=Yellow",
                        f"seq={latest_seq} rssi={latest_rssi} len={latest_len} fmt={latest_version}",
                        f"{disparity_text}  {distance_text}",
                        f"mode={stereo_info}",
                    ]
                    if camera_serial is not None:
                        status_lines.append(f"serial={latest_cam_serial[:64]}")
                    status_lines.append("Press q to quit")
                    draw_status_box(stereo_panel, status_lines, x=10, y=34, line_h=22)
                    canvas = stereo_panel
                else:
                    fallback = np.zeros((CANVAS_H, CANVAS_W, 3), dtype=np.uint8)
                    draw_overlay(fallback, "Stereo camera read failed", 40)
                    draw_wave_on_canvas(fallback, fft_vals, color=(255, 255, 0), inset=80)
                    draw_wave_on_canvas(fallback, latest_amp, color=(0, 0, 255), inset=80)
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
                draw_wave_on_canvas(frame, fft_vals, color=(255, 255, 0), inset=80)
                draw_wave_on_canvas(frame, latest_amp, color=(0, 0, 255), inset=80)
                draw_overlay(frame, f"seq={latest_seq} rssi={latest_rssi} len={latest_len} fmt={latest_version}", 92)
                draw_overlay(frame, "Press q to quit", 118)
                canvas = frame
            else:
                draw_wave_on_canvas(wave, fft_vals, color=(255, 255, 0), inset=20)
                draw_wave_on_canvas(wave, latest_amp, color=(0, 0, 255), inset=20)
                draw_overlay(wave, "Combined CSI Waves: RAW=Red FFT=Yellow", 26)
                draw_overlay(wave, f"CSI seq: {latest_seq}", 52)
                draw_overlay(wave, f"RSSI: {latest_rssi} dBm", 78)
                draw_overlay(wave, f"Raw len: {latest_len}", 104)
                draw_overlay(wave, f"Format: {latest_version}", 130)
                draw_overlay(wave, "Press q to quit", 156)
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
        if camera_serial is not None:
            camera_serial.close()
        if cam_left is not None:
            cam_left.release()
        if cam_right is not None:
            cam_right.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
