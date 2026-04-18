"""
Brio 105 camera feed viewer.
Target device address: 0000.0014.0000.003.000.000.000.000.000

Requires: pip install opencv-python
"""

import argparse
import cv2 
import sys
import time
from typing import Optional

TARGET_ADDRESS = "0000.0014.0000.003.000.000.000.000.000"
WINDOW_NAME = "Brio 105 Camera Feed"
WARMUP_ATTEMPTS = 30  # MSMF on Windows needs a few frames to stabilise


def camera_backends():
    if sys.platform == "darwin":
        return [cv2.CAP_AVFOUNDATION, cv2.CAP_ANY]
    if sys.platform.startswith("win"):
        return [cv2.CAP_DSHOW, cv2.CAP_MSMF, cv2.CAP_ANY]
    return [cv2.CAP_ANY]


def open_camera(index: int) -> Optional[cv2.VideoCapture]:
    """Try to open a camera and warm it up; return cap on success, None on failure."""
    cap = None
    for backend in camera_backends():
        candidate = cv2.VideoCapture(index, backend)
        if candidate.isOpened():
            cap = candidate
            break
        candidate.release()
    if cap is None:
        return None

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    cap.set(cv2.CAP_PROP_FPS, 30)

    # Warm-up: drain initial bad frames that MSMF sometimes emits
    for _ in range(WARMUP_ATTEMPTS):
        ret, _ = cap.read()
        if ret:
            return cap
        time.sleep(0.05)

    cap.release()
    return None


def find_working_camera(max_devices: int = 5) -> Optional[cv2.VideoCapture]:
    """Scan indices starting at 1 (Brio 105) then 0 (built-in), etc."""
    for index in range(max_devices - 1, -1, -1):  # try higher indices first
        print(f"  Trying index {index}...")
        cap = open_camera(index)
        if cap is not None:
            return cap
    return None


def main() -> None:
    parser = argparse.ArgumentParser(description="Open camera feed viewer")
    parser.add_argument("--index", type=int, default=-1, help="Camera index override")
    args = parser.parse_args()

    print(f"Target device address: {TARGET_ADDRESS}")
    print("Opening Brio 105 camera...")

    preferred_index = args.index
    if preferred_index < 0:
        preferred_index = 0 if sys.platform == "darwin" else 1

    cap = open_camera(preferred_index)
    if cap is None:
        print(f"Index {preferred_index} failed. Scanning all cameras...")
        cap = find_working_camera()

    if cap is None:
        print("No working camera found. Make sure the Brio 105 is connected.")
        sys.exit(1)

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"Camera ready: {actual_w}x{actual_h} @ {actual_fps:.1f} fps")
    print("Press 'q' or ESC to quit.")

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

    consecutive_failures = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            consecutive_failures += 1
            if consecutive_failures > 10:
                print("Too many read failures. Camera may have disconnected.")
                break
            time.sleep(0.05)
            continue

        consecutive_failures = 0
        cv2.imshow(WINDOW_NAME, frame)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):  # 'q' or ESC
            break
        if cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
            break  # user closed the window via the X button

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
