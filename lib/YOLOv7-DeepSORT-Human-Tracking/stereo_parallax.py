import argparse
import math
import sys
import time
from typing import Dict, List, Optional, Tuple, Union

import cv2
import numpy as np

from api.yolo import YOLOPersonDetector

# Default to repo checkpoint path
YOLO_MODEL = "./checkpoints/yolov7x.pt"


def camera_backends():
    if sys.platform == "darwin":
        return [cv2.CAP_AVFOUNDATION, cv2.CAP_ANY]
    if sys.platform.startswith("win"):
        return [cv2.CAP_DSHOW, cv2.CAP_MSMF, cv2.CAP_ANY]
    return [cv2.CAP_ANY]


def parse_source(source: str) -> Union[str, int]:
    source = source.strip()
    try:
        return int(source)
    except ValueError:
        return source


def open_video_source(source: Union[str, int]) -> Optional[cv2.VideoCapture]:
    if isinstance(source, int):
        for backend in camera_backends():
            cap = cv2.VideoCapture(source, backend)
            if cap.isOpened():
                return cap
            cap.release()
        return None

    cap = cv2.VideoCapture(source, cv2.CAP_ANY)
    if cap.isOpened():
        return cap
    cap.release()
    return None


def dets_to_people(frame: np.ndarray, dets: np.ndarray) -> List[Dict]:
    people = []
    if dets is None or len(dets) == 0:
        return people

    h, w = frame.shape[:2]
    for det in dets:
        x1, y1, x2, y2, conf = det[:5]
        x1 = int(max(0, min(w - 1, x1)))
        y1 = int(max(0, min(h - 1, y1)))
        x2 = int(max(0, min(w - 1, x2)))
        y2 = int(max(0, min(h - 1, y2)))
        if x2 <= x1 or y2 <= y1:
            continue

        crop = frame[y1:y2, x1:x2]
        people.append(
            {
                "bbox": (x1, y1, x2, y2),
                "conf": float(conf),
                "cx": (x1 + x2) * 0.5,
                "cy": (y1 + y2) * 0.5,
                "h": float(y2 - y1),
                "hist": appearance_hist(crop),
            }
        )
    return people


def appearance_hist(crop: np.ndarray) -> np.ndarray:
    if crop.size == 0:
        return np.zeros((32,), dtype=np.float32)
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    hist = cv2.calcHist([hsv], [0, 1], None, [16, 8], [0, 180, 0, 256]).flatten().astype(np.float32)
    norm = np.linalg.norm(hist) + 1e-6
    return hist / norm


def draw_box(frame: np.ndarray, bbox: Tuple[int, int, int, int], color: Tuple[int, int, int], label: str):
    x1, y1, x2, y2 = bbox
    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
    cv2.putText(
        frame,
        label,
        (x1, max(20, y1 - 8)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        color,
        2,
        cv2.LINE_AA,
    )


def build_matches(left_people: List[Dict], right_people: List[Dict], frame_h: int, cost_th: float) -> List[Tuple[int, int, float]]:
    if not left_people or not right_people:
        return []

    costs = []
    for i, lp in enumerate(left_people):
        for j, rp in enumerate(right_people):
            dy = abs(lp["cy"] - rp["cy"]) / max(1.0, frame_h)
            dh = abs(lp["h"] - rp["h"]) / max(1.0, max(lp["h"], rp["h"]))
            app = float(1.0 - np.dot(lp["hist"], rp["hist"]))
            disp = lp["cx"] - rp["cx"]
            disp_penalty = 0.4 if disp < 1.0 else 0.0
            cost = 0.45 * dy + 0.25 * dh + 0.25 * app + 0.05 * disp_penalty
            costs.append((cost, i, j))

    costs.sort(key=lambda x: x[0])
    used_l = set()
    used_r = set()
    matches = []
    for cost, i, j in costs:
        if cost > cost_th:
            continue
        if i in used_l or j in used_r:
            continue
        used_l.add(i)
        used_r.add(j)
        matches.append((i, j, cost))
    return matches


class GlobalTrackManager:
    def __init__(self, max_age: int = 25, match_dist: float = 0.22) -> None:
        self.next_id = 1
        self.max_age = max_age
        self.match_dist = match_dist
        self.tracks: Dict[int, Dict] = {}
        self.frame_i = 0

    def _feature(self, cx: float, cy: float, w: int, h: int, z: Optional[float]) -> np.ndarray:
        zn = 0.0 if z is None else min(20.0, z) / 20.0
        return np.array([cx / max(1, w), cy / max(1, h), zn], dtype=np.float32)

    def assign(self, cx: float, cy: float, w: int, h: int, z: Optional[float]) -> int:
        self.frame_i += 1
        f = self._feature(cx, cy, w, h, z)

        best_id = None
        best_d = 1e9
        for tid, t in self.tracks.items():
            if self.frame_i - t["seen"] > self.max_age:
                continue
            d = float(np.linalg.norm(f - t["f"]))
            if d < best_d:
                best_d = d
                best_id = tid

        if best_id is not None and best_d <= self.match_dist:
            self.tracks[best_id]["f"] = 0.7 * self.tracks[best_id]["f"] + 0.3 * f
            self.tracks[best_id]["seen"] = self.frame_i
            return best_id

        tid = self.next_id
        self.next_id += 1
        self.tracks[tid] = {"f": f, "seen": self.frame_i}
        return tid

    def gc(self) -> None:
        dead = [tid for tid, t in self.tracks.items() if self.frame_i - t["seen"] > self.max_age]
        for tid in dead:
            del self.tracks[tid]


def main():
    parser = argparse.ArgumentParser(description="Stereo fusion: unified IDs + parallax distance")
    parser.add_argument("--source-left", default="0", help='Left camera source (e.g. "0")')
    parser.add_argument("--source-right", default="1", help='Right camera source (e.g. "1")')
    parser.add_argument("--baseline-m", type=float, default=0.20, help="Camera baseline in meters")
    parser.add_argument("--focal-px", type=float, default=0.0, help="Focal length in pixels (preferred)")
    parser.add_argument("--hfov-deg", type=float, default=78.0, help="Used only if focal-px is 0")
    parser.add_argument("--pair-cost-th", type=float, default=0.60, help="Cross-camera pair match threshold")
    parser.add_argument("--weights", default=YOLO_MODEL, help="YOLO checkpoint path")
    parser.add_argument("--log-file", default="", help="Optional path to save stereo logs as CSV-like lines")
    parser.add_argument("--log-interval", type=float, default=0.20, help="Minimum seconds between console logs")
    args = parser.parse_args()

    src_left = parse_source(args.source_left)
    src_right = parse_source(args.source_right)

    cap_left = open_video_source(src_left)
    cap_right = open_video_source(src_right)
    if cap_left is None:
        raise RuntimeError(f"Could not open left source: {src_left}")
    if cap_right is None:
        raise RuntimeError(f"Could not open right source: {src_right}")

    detector = YOLOPersonDetector()
    detector.load(args.weights)
    track_mgr = GlobalTrackManager()
    last_log_time = 0.0
    frame_seq = 0
    log_fh = open(args.log_file, "a", encoding="utf-8") if args.log_file else None

    print("Stereo mode running. Press q to quit.")
    print(f"Left={src_left}, Right={src_right}, baseline={args.baseline_m} m")
    if log_fh is not None:
        log_fh.write("ts,frame,gid,disparity_px,z_m,d_left_m,d_right_m,cx_left,cx_right\n")
        log_fh.flush()

    try:
        while True:
            ok_l, frame_l = cap_left.read()
            ok_r, frame_r = cap_right.read()
            if not ok_l or not ok_r:
                print("Camera read failed.")
                break
            frame_seq += 1

            rgb_l = cv2.cvtColor(frame_l, cv2.COLOR_BGR2RGB)
            rgb_r = cv2.cvtColor(frame_r, cv2.COLOR_BGR2RGB)

            det_l = detector.detect(rgb_l)
            det_r = detector.detect(rgb_r)
            people_l = dets_to_people(frame_l, det_l)
            people_r = dets_to_people(frame_r, det_r)
            matches = build_matches(people_l, people_r, frame_l.shape[0], args.pair_cost_th)

            focal_px = args.focal_px
            if focal_px <= 0:
                width = frame_l.shape[1]
                focal_px = (width * 0.5) / math.tan(math.radians(args.hfov_deg * 0.5))
            cx0 = frame_l.shape[1] * 0.5

            for i, j, _ in matches:
                lp = people_l[i]
                rp = people_r[j]
                disparity = lp["cx"] - rp["cx"]

                z = None
                d_left = None
                d_right = None
                if disparity > 1.0:
                    z = (args.baseline_m * focal_px) / disparity
                    x_left = ((lp["cx"] - cx0) * z) / focal_px
                    x_right = x_left - args.baseline_m
                    d_left = math.hypot(x_left, z)
                    d_right = math.hypot(x_right, z)

                gid = track_mgr.assign(lp["cx"], lp["cy"], frame_l.shape[1], frame_l.shape[0], z)
                text = f"GID {gid} d={disparity:.1f}px"
                if z is not None and d_left is not None and d_right is not None:
                    text += f" Z={z:.2f}m L={d_left:.2f}m R={d_right:.2f}m"
                ts = time.time()
                if ts - last_log_time >= args.log_interval:
                    log_line = (
                        f"{ts:.3f},{frame_seq},{gid},{disparity:.2f},"
                        f"{'' if z is None else f'{z:.3f}'},"
                        f"{'' if d_left is None else f'{d_left:.3f}'},"
                        f"{'' if d_right is None else f'{d_right:.3f}'},"
                        f"{lp['cx']:.2f},{rp['cx']:.2f}"
                    )
                    print(f"[stereo] {log_line}")
                    if log_fh is not None:
                        log_fh.write(log_line + "\n")
                        log_fh.flush()
                    last_log_time = ts

                draw_box(frame_l, lp["bbox"], (0, 255, 255), text)
                draw_box(frame_r, rp["bbox"], (0, 255, 255), f"GID {gid}")

            # Unmatched boxes are marked separately to show no stereo pair this frame.
            matched_l = {i for i, _, _ in matches}
            matched_r = {j for _, j, _ in matches}
            for i, p in enumerate(people_l):
                if i not in matched_l:
                    draw_box(frame_l, p["bbox"], (100, 100, 255), "UNMATCHED")
            for j, p in enumerate(people_r):
                if j not in matched_r:
                    draw_box(frame_r, p["bbox"], (100, 100, 255), "UNMATCHED")

            cv2.putText(
                frame_l,
                f"Matched pairs: {len(matches)}  Left dets: {len(people_l)}  Right dets: {len(people_r)}",
                (12, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.62,
                (50, 255, 50),
                2,
                cv2.LINE_AA,
            )

            h = min(frame_l.shape[0], frame_r.shape[0])
            frame_l2 = cv2.resize(frame_l, (int(frame_l.shape[1] * h / frame_l.shape[0]), h))
            frame_r2 = cv2.resize(frame_r, (int(frame_r.shape[1] * h / frame_r.shape[0]), h))
            canvas = np.hstack([frame_l2, frame_r2])
            cv2.imshow("Stereo Parallax Distance", canvas)
            track_mgr.gc()

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
    finally:
        cap_left.release()
        cap_right.release()
        if log_fh is not None:
            log_fh.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
