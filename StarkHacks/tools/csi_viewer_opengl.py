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
    parser.add_argument("--bins", type=int, default=64, help="Number of CSI magnitude bins to display")
    parser.add_argument("--history", type=int, default=240, help="Waterfall rows")
    parser.add_argument("--webcam", default="", help="Optional webcam index or path")
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
    # Expected format:
    # CSI,<seq>,<rssi>,<raw_len>,<bins>,<mag_0>,...,<mag_n>
    if not line.startswith("CSI,"):
        return None

    parts = line.strip().split(",")
    if len(parts) < 6:
        return None

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
    return seq, rssi, raw_len, bins, mags


def draw_overlay(img, text, y):
    cv2.putText(img, text, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(img, text, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (20, 20, 20), 1, cv2.LINE_AA)


def main():
    args = parse_args()
    args.bins = max(8, args.bins)
    args.history = max(64, args.history)

    ser = serial.Serial(args.port, args.baud, timeout=0.03)
    cam = open_camera(args.webcam)

    waterfall = np.zeros((args.history, args.bins), dtype=np.uint8)
    latest_seq = 0
    latest_rssi = -127
    latest_len = 0
    fps_counter = 0
    fps_time = time.time()
    fps = 0.0

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
                latest_seq, latest_rssi, latest_len, bins, mags = parsed
                row = np.zeros((args.bins,), dtype=np.uint8)
                clipped = np.array(mags[: args.bins], dtype=np.float32)
                if clipped.size > 0:
                    vmax = max(32.0, float(np.percentile(clipped, 98)))
                    clipped = np.clip((clipped / vmax) * 255.0, 0, 255)
                    row[: clipped.shape[0]] = clipped.astype(np.uint8)

                waterfall = np.roll(waterfall, -1, axis=0)
                waterfall[-1, :] = row

            heatmap = cv2.applyColorMap(waterfall, cv2.COLORMAP_TURBO)
            heatmap = cv2.resize(heatmap, (900, 700), interpolation=cv2.INTER_LINEAR)

            draw_overlay(heatmap, f"CSI seq: {latest_seq}", 26)
            draw_overlay(heatmap, f"RSSI: {latest_rssi} dBm", 52)
            draw_overlay(heatmap, f"Raw len: {latest_len}", 78)
            draw_overlay(heatmap, "Press q to quit", 104)

            if cam is not None:
                ok, frame = cam.read()
                if ok:
                    frame = cv2.resize(frame, (360, 700), interpolation=cv2.INTER_AREA)
                else:
                    frame = np.zeros((700, 360, 3), dtype=np.uint8)
                    draw_overlay(frame, "Webcam read failed", 40)
                canvas = np.hstack([heatmap, frame])
            else:
                canvas = heatmap

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
        if cam is not None:
            cam.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
