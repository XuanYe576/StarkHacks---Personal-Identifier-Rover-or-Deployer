"""
YOLO Tracker: Person detection and tracking on USB webcam

Uses Ultralytics YOLO11 with ByteTrack for real-time
multi-person tracking. Outputs position estimates for fusion.

Dependencies:
    - opencv-python
    - ultralytics>=8.3.0
    - numpy

Author: CSI Tracking System
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Standalone functions
# ---------------------------------------------------------------------------

def initialize_camera(
    camera_id: int = 0,
    resolution: Tuple[int, int] = (640, 480)
) -> cv2.VideoCapture:
    """Initialize USB webcam capture with specified resolution and FPS.

    Attempts to open the camera device and configure capture properties.
    Falls back gracefully if the requested resolution is not supported.

    Args:
        camera_id: OpenCV camera device index (default: 0).
        resolution: Desired (width, height) in pixels.

    Returns:
        Configured ``cv2.VideoCapture`` instance.

    Raises:
        RuntimeError: If the camera cannot be opened.
    """
    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        raise RuntimeError(
            f"Failed to open camera with device ID {camera_id}. "
            "Check that the camera is connected and not in use by another process."
        )

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])
    cap.set(cv2.CAP_PROP_FPS, 30)

    # Log actual capture parameters (may differ from requested)
    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    logger.info(
        "Camera opened: %dx%d @ %.1f fps (requested %dx%d@30)",
        actual_w, actual_h, actual_fps, resolution[0], resolution[1]
    )
    return cap


def run_yolo_detection(
    frame: np.ndarray,
    model: Any,
    conf: float = 0.5
) -> List[Dict[str, Any]]:
    """Run YOLO tracking inference on a single frame.

    Uses Ultralytics YOLO with ByteTrack (via ``model.track``) to detect
    and track persons.  Extracts bounding boxes, confidence scores, track
    IDs, and computes the bottom-centre pixel coordinate for each detection.

    Args:
        frame: BGR image array (H x W x 3).
        model: Loaded Ultralytics YOLO model instance.
        conf: Minimum confidence threshold (default 0.5).

    Returns:
        List of detection dictionaries with keys:
        ``bbox`` ([x1, y1, x2, y2]), ``confidence`` (float),
        ``bottom_center`` ([x, y]), ``track_id`` (int).
    """
    if frame is None or frame.size == 0:
        logger.warning("Empty frame passed to run_yolo_detection; returning no detections.")
        return []

    results = model.track(
        frame,
        persist=True,
        conf=conf,
        classes=[0],          # COCO class 0 = person
        verbose=False
    )

    detections: List[Dict[str, Any]] = []

    if results is None or len(results) == 0:
        return detections

    result = results[0]  # Single-image inference
    if result.boxes is None or result.boxes.id is None:
        return detections

    boxes = result.boxes.xyxy.cpu().numpy()           # N x 4
    confidences = result.boxes.conf.cpu().numpy()       # N
    track_ids = result.boxes.id.cpu().numpy().astype(np.int32)  # N

    for box, conf_val, tid in zip(boxes, confidences, track_ids):
        x1, y1, x2, y2 = box.tolist()
        bottom_center = [(x1 + x2) / 2.0, float(y2)]
        detections.append({
            "bbox": [float(x1), float(y1), float(x2), float(y2)],
            "confidence": float(conf_val),
            "bottom_center": bottom_center,
            "track_id": int(tid),
        })

    return detections


def pixel_to_world(
    pixel: np.ndarray,
    homography: Optional[np.ndarray] = None
) -> np.ndarray:
    """Convert pixel coordinates to world coordinates using a homography matrix.

    Args:
        pixel: Array of pixel coordinates, shape ``(..., 2)``.
        homography: Optional 3x3 homography matrix.  If *None*, pixels are
            returned unchanged (uncalibrated passthrough mode).

    Returns:
        World coordinates with the same batch shape as *pixel*.
    """
    pixel = np.asarray(pixel, dtype=np.float32)
    if homography is None:
        return pixel

    if homography.shape != (3, 3):
        raise ValueError(
            f"Homography matrix must be 3x3, got shape {homography.shape}"
        )

    # Reshape to (N, 2) for perspectiveTransform
    original_shape = pixel.shape
    pts = pixel.reshape(-1, 1, 2).astype(np.float32)
    world = cv2.perspectiveTransform(pts, homography.astype(np.float64))
    return world.reshape(original_shape)


# ---------------------------------------------------------------------------
# Configuration dataclass (mirrors spec from config.py)
# ---------------------------------------------------------------------------

@dataclass
class YOLOConfig:
    """Configuration container for the YOLO tracker."""
    model: str = "yolo11n.pt"
    conf_thresh: float = 0.5
    classes: Tuple[int, ...] = (0,)
    track_buffer: int = 30
    camera_id: int = 0
    resolution: Tuple[int, int] = (640, 480)


# ---------------------------------------------------------------------------
# YOLOTracker class
# ---------------------------------------------------------------------------

class YOLOTracker:
    """YOLO + ByteTrack person tracker for Logitech Brio 105 (or any USB webcam).

    Manages camera capture in a background thread, runs YOLO11 inference
    with ByteTrack, and serves the latest tracks and annotated frames to
    downstream consumers (e.g. the sensor-fusion pipeline).

    Typical usage::

        config = YOLOConfig(model="yolo11n.pt", camera_id=0)
        tracker = YOLOTracker(config)
        tracker.start()
        ...
        tracks = tracker.get_tracks()
        annotated = tracker.get_frame_with_tracks()
        ...
        tracker.stop()
    """

    def __init__(self, config: YOLOConfig) -> None:
        """Load the YOLO model and store configuration.

        Args:
            config: ``YOLOConfig`` instance with model path, camera settings,
                and detection thresholds.
        """
        self.config = config
        self._cap: Optional[cv2.VideoCapture] = None
        self._model: Optional[Any] = None
        self._latest_frame: Optional[np.ndarray] = None
        self._annotated_frame: Optional[np.ndarray] = None
        self._tracks: List[Dict[str, Any]] = []
        self._homography: Optional[np.ndarray] = None

        self._running = False
        self._capture_thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()

        # Load model eagerly so we fail fast on import / weight errors
        try:
            from ultralytics import YOLO
            self._model = YOLO(config.model)
            logger.info("YOLO model loaded: %s", config.model)
        except Exception as exc:
            logger.error("Failed to load YOLO model '%s': %s", config.model, exc)
            raise

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self) -> None:
        """Open the camera and start the background capture + inference thread."""
        if self._running:
            logger.warning("Tracker already running; ignoring start() call.")
            return

        self._cap = initialize_camera(self.config.camera_id, self.config.resolution)
        self._running = True
        self._capture_thread = threading.Thread(target=_capture_loop, args=(self,), daemon=True)
        self._capture_thread.start()
        logger.info("YOLOTracker capture thread started (camera %d).", self.config.camera_id)

    def stop(self) -> None:
        """Stop the capture thread and release the camera."""
        self._running = False
        if self._capture_thread is not None:
            self._capture_thread.join(timeout=3.0)
            self._capture_thread = None
        if self._cap is not None:
            self._cap.release()
            self._cap = None
        logger.info("YOLOTracker stopped.")

    # ------------------------------------------------------------------
    # Accessors (thread-safe)
    # ------------------------------------------------------------------

    def get_tracks(self) -> List[Dict[str, Any]]:
        """Return the most recent list of tracked person detections.

        Each entry contains ``bbox``, ``confidence``, ``bottom_center``,
        and ``track_id``.
        """
        with self._lock:
            return list(self._tracks)

    def get_frame_with_tracks(self) -> Optional[np.ndarray]:
        """Return the most recent annotated frame (or *None* if not ready).

        The frame has bounding boxes and track IDs drawn by the Ultralytics
        ``plot`` helper.
        """
        with self._lock:
            return self._annotated_frame.copy() if self._annotated_frame is not None else None

    # ------------------------------------------------------------------
    # Calibration
    # ------------------------------------------------------------------

    def get_homography_matrix(
        self,
        pixel_points: np.ndarray,
        world_points: np.ndarray
    ) -> np.ndarray:
        """Compute a pixel-to-world homography matrix from calibration points.

        Requires at least 4 corresponding (pixel, world) point pairs.

        Args:
            pixel_points: Nx2 array of pixel coordinates.
            world_points: Nx2 array of corresponding world coordinates.

        Returns:
            3x3 homography matrix.

        Raises:
            ValueError: If fewer than 4 points are supplied.
        """
        pixel_points = np.asarray(pixel_points, dtype=np.float32)
        world_points = np.asarray(world_points, dtype=np.float32)

        if pixel_points.shape[0] < 4 or world_points.shape[0] < 4:
            raise ValueError(
                "At least 4 point pairs are required to compute homography "
                f"(got {pixel_points.shape[0]})."
            )

        H, status = cv2.findHomography(pixel_points, world_points, cv2.RANSAC)
        if H is None:
            raise RuntimeError("cv2.findHomography failed to compute a valid H matrix.")

        inliers = int(status.sum()) if status is not None else pixel_points.shape[0]
        logger.info("Homography computed: %d/%d inliers.", inliers, pixel_points.shape[0])

        self._homography = H.astype(np.float64)
        return self._homography

    # ------------------------------------------------------------------
    # Internal helpers (called from _capture_loop)
    # ------------------------------------------------------------------

    def _update(self, frame: np.ndarray) -> None:
        """Run inference on *frame* and cache results (called by worker thread)."""
        if self._model is None:
            return

        detections = run_yolo_detection(
            frame, self._model, conf=self.config.conf_thresh
        )

        # Convert bottom_center pixels to world coordinates if calibrated
        if self._homography is not None:
            for det in detections:
                bc = np.array(det["bottom_center"], dtype=np.float32).reshape(1, 1, 2)
                world_bc = cv2.perspectiveTransform(bc, self._homography)
                det["world_position"] = world_bc.reshape(2).tolist()

        # Build annotated frame
        annotated = frame.copy()
        for det in detections:
            x1, y1, x2, y2 = map(int, det["bbox"])
            tid = det["track_id"]
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f"ID:{tid} {det['confidence']:.2f}"
            cv2.putText(
                annotated, label, (x1, max(y1 - 10, 20)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2
            )
            # Draw bottom-center point
            bc = tuple(map(int, det["bottom_center"]))
            cv2.circle(annotated, bc, 4, (0, 0, 255), -1)

        with self._lock:
            self._tracks = detections
            self._latest_frame = frame
            self._annotated_frame = annotated


# ---------------------------------------------------------------------------
# Background worker
# ---------------------------------------------------------------------------

def _capture_loop(tracker: YOLOTracker) -> None:
    """Continuously grab frames and run YOLO inference.

    Runs inside the tracker daemon thread until ``tracker._running`` becomes
    *False*.
    """
    missed_frames = 0
    while tracker._running:
        if tracker._cap is None:
            time.sleep(0.01)
            continue

        ret, frame = tracker._cap.read()
        if not ret:
            missed_frames += 1
            if missed_frames % 30 == 0:
                logger.warning("Camera read failed %d consecutive times.", missed_frames)
            time.sleep(0.001)
            continue

        missed_frames = 0
        try:
            tracker._update(frame)
        except Exception as exc:
            logger.error("Error in YOLO inference: %s", exc)

    logger.debug("Capture loop exiting.")


# ---------------------------------------------------------------------------
# Self-test
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import sys

    logging.basicConfig(
        level=logging.DEBUG,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s"
    )

    # Quick sanity checks without requiring a camera
    print("=" * 60)
    print("yolo_tracker.py self-test")
    print("=" * 60)

    # 1. pixel_to_world passthrough
    pts = np.array([[100.0, 200.0], [300.0, 400.0]])
    out = pixel_to_world(pts, homography=None)
    assert np.allclose(out, pts), "pixel_to_world passthrough failed"
    print("[PASS] pixel_to_world passthrough (no homography)")

    # 2. pixel_to_world with identity homography
    H_identity = np.eye(3, dtype=np.float64)
    out_id = pixel_to_world(pts, homography=H_identity)
    assert np.allclose(out_id, pts), "pixel_to_world identity failed"
    print("[PASS] pixel_to_world identity homography")

    # 3. pixel_to_world with translation homography
    H_translate = np.eye(3, dtype=np.float64)
    H_translate[0, 2] = 5.0   # x + 5
    H_translate[1, 2] = -3.0  # y - 3
    out_tr = pixel_to_world(pts, homography=H_translate)
    expected = pts + np.array([5.0, -3.0])
    assert np.allclose(out_tr, expected), "pixel_to_world translation failed"
    print("[PASS] pixel_to_world translation homography")

    # 4. pixel_to_world invalid homography shape
    try:
        pixel_to_world(pts, homography=np.eye(4))
        print("[FAIL] Expected ValueError for invalid H shape")
    except ValueError:
        print("[PASS] pixel_to_world rejects invalid homography shape")

    # 5. YOLOConfig dataclass
    cfg = YOLOConfig(model="yolo11n.pt", camera_id=0)
    assert cfg.conf_thresh == 0.5
    assert cfg.resolution == (640, 480)
    print("[PASS] YOLOConfig defaults")

    # 6. Tracker instantiation (without camera)
    try:
        tracker = YOLOTracker(cfg)
        print("[PASS] YOLOTracker instantiation (model loaded)")

        # Test get_homography_matrix
        pix = np.array([[0, 0], [640, 0], [640, 480], [0, 480]], dtype=np.float32)
        wld = np.array([[0, 0], [4, 0], [4, 3], [0, 3]], dtype=np.float32)
        H = tracker.get_homography_matrix(pix, wld)
        assert H.shape == (3, 3)
        print("[PASS] get_homography_matrix 4-point calibration")

        # Test get_homography_matrix with < 4 points (should raise)
        try:
            tracker.get_homography_matrix(pix[:3], wld[:3])
            print("[FAIL] Expected ValueError for < 4 points")
        except ValueError:
            print("[PASS] get_homography_matrix rejects < 4 points")

    except ImportError as exc:
        print(f"[SKIP] Ultralytics not installed ({exc})")
    except Exception as exc:
        print(f"[WARN] Tracker instantiation: {exc}")

    print("=" * 60)
    print("Self-test complete.")
    print("=" * 60)
