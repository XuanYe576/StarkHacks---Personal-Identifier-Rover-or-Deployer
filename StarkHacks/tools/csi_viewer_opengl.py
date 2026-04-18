#!/usr/bin/env python3
import argparse
import sys
import time

import cv2
import numpy as np
import serial


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
    parser.add_argument("--stereo-detect", action="store_true", help="Enable built-in people detect/parallax overlay")
    parser.add_argument("--stereo-detect-interval", type=int, default=6, help="Run detector every N frames")
    parser.add_argument("--stereo-baseline-m", type=float, default=0.20, help="Stereo baseline in meters")
    parser.add_argument("--stereo-focal-px", type=float, default=0.0, help="Calibrated focal length in pixels")
    parser.add_argument("--stereo-hfov-deg", type=float, default=78.0, help="HFOV used when focal-px is 0")
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


def render_stereo_panel(frame_left, frame_right, info_line):
    left = fit_frame_with_padding(frame_left, 450, 680)
    right = fit_frame_with_padding(frame_right, 450, 680)
    panel = np.zeros((680, 900, 3), dtype=np.uint8)
    # Requested order: [cam1][cam0]
    panel[:, 0:450, :] = right
    panel[:, 450:900, :] = left
    cv2.line(panel, (450, 0), (450, 679), (50, 50, 50), 1)
    draw_overlay(panel, "CAM1", 24)
    cv2.putText(panel, "CAM0", (462, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(panel, "CAM0", (462, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (20, 20, 20), 1, cv2.LINE_AA)
    cv2.putText(panel, info_line, (12, 654), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(panel, info_line, (12, 654), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (20, 20, 20), 1, cv2.LINE_AA)
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

    ser = serial.Serial(args.port, args.baud, timeout=0.03)
    cam_left = open_camera(args.webcam)
    cam_right = open_camera(args.stereo_right) if args.stereo_right else None
    hog = None
    if args.stereo_detect:
        hog = cv2.HOGDescriptor()
        hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
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
    detect_frame_i = 0
    cached_left_box = None
    cached_right_box = None

    window_name = "CSI OpenGL Viewer"
    try:
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL | cv2.WINDOW_OPENGL)
    except cv2.error:
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 1280, 720)

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
            wave = np.zeros((680, 900, 3), dtype=np.uint8)

            if cam_left is not None and cam_right is not None:
                ok_l, frame_l = cam_left.read()
                ok_r, frame_r = cam_right.read()
                if ok_l and ok_r:
                    detect_frame_i += 1
                    if hog is not None and (detect_frame_i % args.stereo_detect_interval == 0):
                        cached_left_box = detect_primary_person(frame_l, hog)
                        cached_right_box = detect_primary_person(frame_r, hog)

                    if cached_left_box is not None:
                        x, y, w, h = cached_left_box
                        cv2.rectangle(frame_l, (x, y), (x + w, y + h), (0, 255, 255), 2)
                    if cached_right_box is not None:
                        x, y, w, h = cached_right_box
                        cv2.rectangle(frame_r, (x, y), (x + w, y + h), (0, 255, 255), 2)

                    stereo_info = "stereo on"
                    if cached_left_box is not None and cached_right_box is not None:
                        cx_l = cached_left_box[0] + (cached_left_box[2] * 0.5)
                        cx_r = cached_right_box[0] + (cached_right_box[2] * 0.5)
                        disparity = cx_l - cx_r
                        if disparity > 1.0:
                            focal_px = args.stereo_focal_px
                            if focal_px <= 0.0:
                                focal_px = (frame_l.shape[1] * 0.5) / np.tan(np.radians(args.stereo_hfov_deg * 0.5))
                            depth = (args.stereo_baseline_m * focal_px) / disparity
                            stereo_info = f"stereo detect ON d={disparity:.1f}px z={depth:.2f}m"
                        else:
                            stereo_info = f"stereo detect ON d={disparity:.1f}px"

                    stereo_panel = render_stereo_panel(frame_l, frame_r, stereo_info[:54])
                    if camera_serial is not None:
                        draw_overlay(stereo_panel, f"Serial: {latest_cam_serial[:34]}", 40)
                    # Overlay CSI graph directly over [cam1][cam0] interface.
                    draw_wave_on_canvas(stereo_panel, fft_vals, color=(255, 255, 0), inset=80)
                    draw_wave_on_canvas(stereo_panel, latest_amp, color=(0, 0, 255), inset=80)
                    draw_overlay(stereo_panel, "CSI overlay: RAW=Red FFT=Yellow", 52)
                    draw_overlay(stereo_panel, f"seq={latest_seq} rssi={latest_rssi} len={latest_len} fmt={latest_version}", 78)
                    if camera_serial is not None:
                        draw_overlay(stereo_panel, f"Camera serial: {latest_cam_serial[:70]}", 104)
                    draw_overlay(stereo_panel, "Press q to quit", 130)
                    canvas = stereo_panel
                else:
                    fallback = np.zeros((680, 900, 3), dtype=np.uint8)
                    draw_overlay(fallback, "Stereo camera read failed", 40)
                    draw_wave_on_canvas(fallback, fft_vals, color=(255, 255, 0), inset=80)
                    draw_wave_on_canvas(fallback, latest_amp, color=(0, 0, 255), inset=80)
                    canvas = fallback
            elif cam_left is not None:
                ok, frame = cam_left.read()
                if ok:
                    frame = fit_frame_with_padding(frame, 900, 680)
                    if camera_serial is not None:
                        draw_overlay(frame, f"Serial: {latest_cam_serial[:48]}", 40)
                    draw_overlay(frame, "Stereo off (set --stereo-right)", 66)
                else:
                    frame = np.zeros((680, 900, 3), dtype=np.uint8)
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
            draw_overlay(canvas, f"Render FPS: {fps:.1f}", 130)

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
