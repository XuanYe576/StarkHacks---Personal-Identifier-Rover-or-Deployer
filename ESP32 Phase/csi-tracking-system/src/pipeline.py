"""
Real-Time Processing Pipeline

Multiprocessing pipeline connecting CSI sources, YOLO tracker,
and EKF fusion. Uses queues for inter-process communication.

Architecture
------------
::

    +-------------+     +-------------+     +-------------+
    | CSI Sources | --> |  CSI Queue  | --> |   Fusion    |
    |   (UDP)     |     |  (mp.Queue) |     |   Thread    |
    +-------------+     +-------------+     +------+------+
    +-------------+     +-------------+            |
    | YOLO Tracker| --> |  YOLO Queue | <----------+
    |  (USB cam)  |     |  (mp.Queue) |
    +-------------+     +-------------+

The fusion thread consumes from both the CSI and YOLO queues, runs the
EKF update/predict cycle, and publishes the fused state.

Dependencies
    - numpy, scipy, opencv-python, matplotlib
    - sibling modules: csi_core, yolo_tracker, sensor_fusion,
      localization, multi_esp_fft, config
"""

from __future__ import annotations

import logging
import multiprocessing as mp
import threading
import time
from typing import Any, Callable, Dict, List, Optional, Tuple

import numpy as np

# Matplotlib is optional for the visualiser
try:
    import matplotlib
    matplotlib.use("Agg")  # Non-interactive backend for headless environments
    import matplotlib.pyplot as plt
    _HAS_MPL = True
except ImportError:
    _HAS_MPL = False

try:
    import cv2
    _HAS_CV2 = True
except ImportError:
    _HAS_CV2 = False

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Lazy imports of sibling modules (graceful degradation when stubs absent)
# ---------------------------------------------------------------------------

_csi_core: Any = None
_yolo_tracker: Any = None
_sensor_fusion: Any = None
_localization: Any = None
_multi_esp_fft: Any = None


def _import_siblings() -> None:
    """Import sibling modules lazily so the pipeline file can be imported
    independently for inspection without pulling in heavy dependencies."""
    global _csi_core, _yolo_tracker, _sensor_fusion, _localization, _multi_esp_fft
    if _csi_core is not None:
        return
    try:
        from src import csi_core as _csi_core
    except Exception as exc:
        logger.debug("csi_core not available: %s", exc)
    try:
        from src import yolo_tracker as _yolo_tracker
    except Exception as exc:
        logger.debug("yolo_tracker not available: %s", exc)
    try:
        from src import sensor_fusion as _sensor_fusion
    except Exception as exc:
        logger.debug("sensor_fusion not available: %s", exc)
    try:
        from src import localization as _localization
    except Exception as exc:
        logger.debug("localization not available: %s", exc)
    try:
        from src import multi_esp_fft as _multi_esp_fft
    except Exception as exc:
        logger.debug("multi_esp_fft not available: %s", exc)


# ---------------------------------------------------------------------------
# TrackingPipeline
# ---------------------------------------------------------------------------

class TrackingPipeline:
    """Orchestrates the real-time CSI + YOLO tracking pipeline.

    The pipeline manages three concurrent activities:

    1. **CSI acquisition** — one background thread per ESP32 that listens
       on a UDP socket and pushes parsed CSI frames into ``queues['csi']``.
    2. **YOLO tracking** — either runs in the main process or a separate
       process (configurable) and pushes person tracks into
       ``queues['yolo']``.
    3. **Fusion** — a background thread that consumes both queues and
       drives the EKF, producing the fused position estimate.

    Args:
        csi_config: ``CSIConfig`` dataclass (ESP32 IP list, port, flags).
        yolo_config: ``YOLOConfig`` dataclass (model path, camera, etc.).
        fusion_config: ``FusionConfig`` dataclass (EKF noise parameters).

    Example::

        pipeline = TrackingPipeline(csi_cfg, yolo_cfg, fusion_cfg)
        pipeline.start()
        while running:
            state = pipeline.get_state()
            print(state["position"])
        pipeline.stop()
    """

    def __init__(
        self,
        csi_config: Any,
        yolo_config: Any,
        fusion_config: Any
    ) -> None:
        _import_siblings()

        self.csi_config = csi_config
        self.yolo_config = yolo_config
        self.fusion_config = fusion_config

        # CSI data sources: esp_id -> CSISource
        self.csi_sources: Dict[str, Any] = {}

        # YOLO tracker instance
        self.yolo: Optional[Any] = None
        if _yolo_tracker is not None:
            self.yolo = _yolo_tracker.YOLOTracker(yolo_config)
        else:
            logger.warning("yolo_tracker module unavailable; YOLO tracking disabled.")

        # EKF fusion instance
        self.ekf: Any = None
        if _sensor_fusion is not None:
            self.ekf = _sensor_fusion.CSI_YOLO_FusionEKF(**fusion_config.__dict__)
        else:
            # Fallback: create a minimal EKF-like stub so get_state() still works
            self.ekf = _MinimalEKFStub(**fusion_config.__dict__)
            logger.warning("sensor_fusion module unavailable; using EKF stub.")

        # Optional multi-ESP localisation helpers
        self.localizer: Optional[Any] = None
        self.multi_esp: Optional[Any] = None

        if getattr(csi_config, "use_multi_esp", False):
            if _localization is not None:
                self.localizer = _localization.LocalizationEngine([])
            if _multi_esp_fft is not None:
                self.multi_esp = _multi_esp_fft.MultiESPProcessor([])
            if self.localizer is None:
                logger.warning("localization module unavailable; multi-ESP localiser stubbed.")
            if self.multi_esp is None:
                logger.warning("multi_esp_fft module unavailable; multi-ESP processor stubbed.")

        # IPC queues
        self.queues: Dict[str, mp.Queue] = {
            "csi": mp.Queue(maxsize=100),
            "yolo": mp.Queue(maxsize=10),
            "fusion": mp.Queue(maxsize=10),
        }

        # Thread / process handles
        self._csi_threads: List[threading.Thread] = []
        self._fusion_thread: Optional[threading.Thread] = None
        self._yolo_process: Optional[mp.Process] = None
        self._yolo_thread: Optional[threading.Thread] = None

        self.running = False
        self._last_wifi_pos: Optional[np.ndarray] = None
        self._last_cam_pos: Optional[np.ndarray] = None
        self._lock = threading.Lock()

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self) -> None:
        """Start all pipeline workers.

        1. Create and start a ``CSISource`` (and optionally
           ``MultiESPProcessor``) for each configured ESP32.
        2. Start the YOLO tracker (in-process thread mode).
        3. Start the EKF fusion consumer thread.
        """
        if self.running:
            logger.warning("Pipeline already running; ignoring start() call.")
            return

        self.running = True
        logger.info("Starting TrackingPipeline ...")

        # ---- 1. CSI acquisition threads ----
        esp_ips = getattr(self.csi_config, "esp32_ip", [])
        if isinstance(esp_ips, str):
            esp_ips = [esp_ips]

        for idx, ip in enumerate(esp_ips):
            esp_id = f"esp_{idx}"
            if _csi_core is not None:
                src = _csi_core.CSISource(ip, self.csi_config.esp32_port)
            else:
                src = _CSISourceStub(ip, self.csi_config.esp32_port)
            self.csi_sources[esp_id] = src
            src.start()
            t = threading.Thread(
                target=self._csi_producer,
                args=(esp_id, src),
                daemon=True,
                name=f"CSI-{esp_id}"
            )
            t.start()
            self._csi_threads.append(t)
            logger.info("CSI producer thread started for %s (%s:%d).", esp_id, ip, self.csi_config.esp32_port)

        # ---- 2. YOLO tracker ----
        if self.yolo is not None:
            self.yolo.start()
            self._yolo_thread = threading.Thread(
                target=self._yolo_producer,
                daemon=True,
                name="YOLO"
            )
            self._yolo_thread.start()
            logger.info("YOLO producer thread started.")

        # ---- 3. Fusion thread ----
        self._fusion_thread = threading.Thread(
            target=self._fusion_consumer,
            daemon=True,
            name="Fusion"
        )
        self._fusion_thread.start()
        logger.info("Fusion consumer thread started.")

        logger.info("TrackingPipeline fully started.")

    def stop(self) -> None:
        """Signal all workers to stop and join threads."""
        if not self.running:
            return
        logger.info("Stopping TrackingPipeline ...")
        self.running = False

        # Join CSI threads
        for t in self._csi_threads:
            t.join(timeout=2.0)
        self._csi_threads.clear()

        # Join YOLO thread
        if self._yolo_thread is not None:
            self._yolo_thread.join(timeout=2.0)
            self._yolo_thread = None

        # Stop YOLO tracker (releases camera)
        if self.yolo is not None:
            self.yolo.stop()

        # Join fusion thread
        if self._fusion_thread is not None:
            self._fusion_thread.join(timeout=2.0)
            self._fusion_thread = None

        # Stop CSI sources
        for src in self.csi_sources.values():
            try:
                src.stop()
            except Exception as exc:
                logger.debug("Error stopping CSI source: %s", exc)
        self.csi_sources.clear()

        # Drain queues so child processes can exit cleanly
        for q in self.queues.values():
            while not q.empty():
                try:
                    q.get_nowait()
                except Exception:
                    break

        logger.info("TrackingPipeline stopped.")

    # ------------------------------------------------------------------
    # State query
    # ------------------------------------------------------------------

    def get_state(self) -> Dict[str, Any]:
        """Return the current fused tracking state.

        Returns:
            Dictionary with keys:

            - ``position`` (List[float] | None): Fused (x, y) position.
            - ``covariance`` (List[List[float]]): 2x2 position covariance.
            - ``tracks`` (List[Dict]): Active YOLO tracks.
            - ``wifi_bias`` (List[float]): Estimated WiFi measurement bias.
            - ``timestamp`` (float): Monotonic time of the state snapshot.
        """
        pos, cov = self.ekf.get_position()
        tracks: List[Dict] = []
        if self.yolo is not None:
            tracks = self.yolo.get_tracks()

        wifi_bias = getattr(self.ekf, "wifi_bias", np.zeros(2))
        if wifi_bias is None:
            wifi_bias = np.zeros(2)

        return {
            "position": pos.tolist() if pos is not None else None,
            "covariance": cov.tolist() if cov is not None else [[0.0, 0.0], [0.0, 0.0]],
            "tracks": tracks,
            "wifi_bias": wifi_bias.tolist(),
            "timestamp": time.monotonic(),
        }

    # ------------------------------------------------------------------
    # Internal worker threads
    # ------------------------------------------------------------------

    def _csi_producer(self, esp_id: str, source: Any) -> None:
        """Pull windows from *source* and push into the CSI queue."""
        logger.debug("CSI producer %s started.", esp_id)
        while self.running:
            try:
                window = source.get_window(
                    getattr(self.csi_config, "window_size_ms", 100)
                )
                if window is not None:
                    payload = {
                        "esp_id": esp_id,
                        "timestamp": time.monotonic(),
                        "data": window,
                    }
                    # Non-blocking push with drop-on-full
                    try:
                        self.queues["csi"].put_nowait(payload)
                    except Exception:
                        # Queue full — drop oldest
                        try:
                            self.queues["csi"].get_nowait()
                            self.queues["csi"].put_nowait(payload)
                        except Exception:
                            pass
            except Exception as exc:
                logger.error("CSI producer %s error: %s", esp_id, exc)
            time.sleep(0.001)
        logger.debug("CSI producer %s exiting.", esp_id)

    def _yolo_producer(self) -> None:
        """Poll the YOLO tracker and push tracks into the YOLO queue."""
        logger.debug("YOLO producer started.")
        while self.running:
            try:
                if self.yolo is None:
                    time.sleep(0.05)
                    continue
                tracks = self.yolo.get_tracks()
                if tracks:
                    payload = {
                        "timestamp": time.monotonic(),
                        "tracks": tracks,
                    }
                    try:
                        self.queues["yolo"].put_nowait(payload)
                    except Exception:
                        try:
                            self.queues["yolo"].get_nowait()
                            self.queues["yolo"].put_nowait(payload)
                        except Exception:
                            pass
            except Exception as exc:
                logger.error("YOLO producer error: %s", exc)
            time.sleep(0.033)  # ~30 FPS
        logger.debug("YOLO producer exiting.")

    def _fusion_consumer(self) -> None:
        """Consume CSI and YOLO queues, run EKF predict/update."""
        logger.debug("Fusion consumer started.")
        last_predict_time = time.monotonic()

        while self.running:
            now = time.monotonic()
            dt = now - last_predict_time

            # ---- EKF predict step ----
            try:
                self.ekf.predict(dt=max(dt, 0.001))
                last_predict_time = now
            except Exception as exc:
                logger.error("EKF predict error: %s", exc)

            # ---- Process CSI measurements ----
            try:
                while not self.queues["csi"].empty():
                    csi_msg = self.queues["csi"].get_nowait()
                    self._handle_csi_message(csi_msg)
            except Exception:
                pass

            # ---- Process YOLO measurements ----
            try:
                while not self.queues["yolo"].empty():
                    yolo_msg = self.queues["yolo"].get_nowait()
                    self._handle_yolo_message(yolo_msg)
            except Exception:
                pass

            time.sleep(0.01)  # 100 Hz fusion loop

        logger.debug("Fusion consumer exiting.")

    # ------------------------------------------------------------------
    # Message handlers
    # ------------------------------------------------------------------

    def _handle_csi_message(self, msg: Dict[str, Any]) -> None:
        """Process a CSI frame and feed the resulting position into the EKF."""
        esp_id = msg.get("esp_id", "unknown")
        data = msg.get("data")

        if data is None:
            return

        wifi_pos: Optional[np.ndarray] = None
        wifi_var: float = getattr(self.fusion_config, "measurement_noise_wifi", 2.0)

        try:
            # Multi-ESP path: process through MultiESPProcessor -> localizer
            if getattr(self.csi_config, "use_multi_esp", False):
                if self.multi_esp is not None:
                    self.multi_esp.process_frame(esp_id, msg)
                    distances = self.multi_esp.get_smoothed_distances()
                    if distances and self.localizer is not None:
                        for eid, dist in distances.items():
                            self.localizer.update_distances(eid, dist, wifi_var)
                        result = self.localizer.estimate_position()
                        if result is not None:
                            wifi_pos, wifi_cov = result
                            wifi_var = float(np.trace(wifi_cov)) / 2.0
                else:
                    # Fallback: amplitude-based detection from CSI data
                    wifi_pos = self._csi_amplitude_position(data)
            else:
                # Single-ESP: amplitude-based presence detection
                wifi_pos = self._csi_amplitude_position(data)

        except Exception as exc:
            logger.debug("CSI message handling error: %s", exc)

        if wifi_pos is not None:
            self._last_wifi_pos = wifi_pos
            try:
                self.ekf.update_wifi(wifi_pos, variance=wifi_var)
            except Exception as exc:
                logger.debug("EKF WiFi update error: %s", exc)

    def _handle_yolo_message(self, msg: Dict[str, Any]) -> None:
        """Process YOLO tracks and feed camera positions into the EKF."""
        tracks = msg.get("tracks", [])
        if not tracks:
            return

        cam_var = getattr(self.fusion_config, "measurement_noise_cam", 0.3)

        # Use the first (highest-confidence) track
        track = tracks[0]
        bottom_center = track.get("bottom_center")
        world_pos = track.get("world_position")

        try:
            if world_pos is not None:
                cam_pos = np.array(world_pos, dtype=np.float64)
            elif bottom_center is not None:
                cam_pos = np.array(bottom_center, dtype=np.float64)
            else:
                return

            self._last_cam_pos = cam_pos
            self.ekf.update_camera(cam_pos, variance=cam_var)

            # Camera-assisted WiFi bias estimation
            if self._last_wifi_pos is not None:
                try:
                    bias = self.ekf.estimate_wifi_bias(cam_pos, self._last_wifi_pos)
                    if bias is not None:
                        self.ekf.wifi_bias = bias
                except Exception as exc:
                    logger.debug("WiFi bias estimation error: %s", exc)

        except Exception as exc:
            logger.debug("YOLO message handling error: %s", exc)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _csi_amplitude_position(csi_data: Any) -> Optional[np.ndarray]:
        """Derive a coarse position estimate from single-CSI amplitude data.

        This is a placeholder that returns a heuristic based on RSSI
        amplitude variance.  In a real deployment this would be replaced
        by a proper AoA or fingerprinting estimator.

        Returns:
            ndarray of shape (2,) or *None* if no estimate is possible.
        """
        try:
            amp = np.asarray(csi_data)
            if amp.ndim >= 2:
                # Use mean amplitude across subcarriers as a simple presence proxy
                mean_amp = np.mean(amp)
                # Map to a dummy position (centre of room as placeholder)
                # In production this would be replaced with real localisation
                return np.array([2.0 + (mean_amp % 1.0), 1.5 + ((mean_amp * 2) % 1.0)])
            return np.array([2.0, 1.5])
        except Exception:
            return None


# ---------------------------------------------------------------------------
# Stubs for graceful degradation when sibling modules are unavailable
# ---------------------------------------------------------------------------

class _MinimalEKFStub:
    """Minimal stand-in for ``CSI_YOLO_FusionEKF`` when sensor_fusion.py is
    not importable.  Maintains the same public interface so *get_state()*
    and the fusion loop never crash."""

    def __init__(self, **kwargs: Any) -> None:
        self.x = np.zeros(4)
        self.P = np.eye(4) * 10.0
        self.Q = np.eye(4) * kwargs.get("process_noise", 0.1)
        self.R_wifi = np.eye(2) * kwargs.get("measurement_noise_wifi", 2.0)
        self.R_cam = np.eye(2) * kwargs.get("measurement_noise_cam", 0.3)
        self.wifi_bias = np.zeros(2)

    def predict(self, dt: float = 0.1) -> None:
        F = np.array([[1, 0, dt, 0],
                      [0, 1, 0, dt],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]], dtype=np.float64)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q

    def update_wifi(self, pos: np.ndarray, variance: Optional[float] = None) -> None:
        self._update_measurement(pos, self.R_wifi if variance is None else np.eye(2) * variance)

    def update_camera(self, pos: np.ndarray, variance: Optional[float] = None) -> None:
        self._update_measurement(pos, self.R_cam if variance is None else np.eye(2) * variance)

    def get_position(self) -> Tuple[np.ndarray, np.ndarray]:
        return self.x[:2].copy(), self.P[:2, :2].copy()

    def estimate_wifi_bias(self, camera_pos: np.ndarray, wifi_pos: np.ndarray) -> np.ndarray:
        return np.array(camera_pos) - np.array(wifi_pos)

    def _update_measurement(self, z: np.ndarray, R: np.ndarray) -> None:
        H = np.array([[1, 0, 0, 0],
                      [0, 1, 0, 0]], dtype=np.float64)
        z = np.asarray(z).reshape(2)
        y = z - H @ self.x
        S = H @ self.P @ H.T + R
        try:
            K = self.P @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            K = self.P @ H.T @ np.linalg.pinv(S)
        self.x = self.x + K @ y
        I_KH = np.eye(4) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T


class _CSISourceStub:
    """Stand-in for ``CSISource`` when csi_core.py is unavailable."""

    def __init__(self, ip: str, port: int) -> None:
        self.ip = ip
        self.port = port
        self._running = False

    def start(self) -> None:
        self._running = True
        logger.debug("CSISourceStub started (%s:%d).", self.ip, self.port)

    def get_window(self, n_samples: int) -> Optional[np.ndarray]:
        if not self._running:
            return None
        # Return synthetic data for testing
        return np.random.randn(n_samples, 52).astype(np.float32)

    def stop(self) -> None:
        self._running = False


# ---------------------------------------------------------------------------
# PipelineVisualizer
# ---------------------------------------------------------------------------

class PipelineVisualizer:
    """Real-time matplotlib-based visualisation of the fused tracking state.

    Produces a top-down room view showing anchor positions, the estimated
    person location (with covariance ellipse), and active YOLO track IDs.
    The rendered image is returned as an RGB NumPy array suitable for
    OpenCV ``imshow``.

    Args:
        room_bounds: Tuple of (xmin, xmax, ymin, ymax) in metres.
        anchor_positions: Optional Nx2 array of ESP32 anchor coordinates.
    """

    def __init__(
        self,
        room_bounds: Tuple[float, float, float, float] = (0.0, 5.0, 0.0, 4.0),
        anchor_positions: Optional[np.ndarray] = None
    ) -> None:
        if not _HAS_MPL:
            raise ImportError(
                "matplotlib is required for PipelineVisualizer. "
                "Install it with: pip install matplotlib"
            )

        self.room_bounds = room_bounds
        self.anchor_positions = anchor_positions
        self.current_state: Dict[str, Any] = {}

        self.fig, self.ax = plt.subplots(1, 1, figsize=(8, 8))
        self.fig.canvas.draw()

    def update(self, state: Dict[str, Any]) -> None:
        """Store the latest tracking state for rendering.

        Args:
            state: Dictionary returned by ``TrackingPipeline.get_state()``.
        """
        self.current_state = state

    def render(self) -> np.ndarray:
        """Draw the tracking scene and return as an RGB array.

        Returns:
            uint8 RGB image array of shape (H, W, 3).
        """
        self.ax.clear()
        xmin, xmax, ymin, ymax = self.room_bounds
        self.ax.set_xlim(xmin, xmax)
        self.ax.set_ylim(ymin, ymax)
        self.ax.set_aspect("equal")
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.set_title("CSI + YOLO Tracking")
        self.ax.grid(True, alpha=0.3)

        # Draw anchors
        if self.anchor_positions is not None and self.anchor_positions.size > 0:
            self.ax.scatter(
                self.anchor_positions[:, 0],
                self.anchor_positions[:, 1],
                c="blue",
                marker="^",
                s=200,
                label="Anchors",
                edgecolors="black"
            )

        # Draw estimated position
        position = self.current_state.get("position")
        covariance = self.current_state.get("covariance")

        if position is not None and len(position) == 2:
            pos = np.asarray(position)
            self.ax.scatter(pos[0], pos[1], c="red", marker="o", s=300, label="Estimate", zorder=5)

            # Covariance ellipse
            if covariance is not None:
                cov = np.asarray(covariance)
                if cov.shape == (2, 2):
                    try:
                        self._draw_covariance_ellipse(pos, cov, n_std=2.0)
                    except Exception as exc:
                        logger.debug("Ellipse draw error: %s", exc)

            # WiFi bias arrow
            wifi_bias = self.current_state.get("wifi_bias")
            if wifi_bias is not None and np.linalg.norm(wifi_bias) > 0.01:
                self.ax.annotate(
                    "", xy=pos + np.array(wifi_bias), xytext=pos,
                    arrowprops=dict(arrowstyle="->", color="orange", lw=2),
                )

            # Track count
            tracks = self.current_state.get("tracks", [])
            self.ax.text(
                0.02, 0.98, f"Tracks: {len(tracks)}\n"
                             f"Pos: ({pos[0]:.2f}, {pos[1]:.2f})",
                transform=self.ax.transAxes,
                verticalalignment="top",
                bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5),
                fontsize=10,
            )

        self.ax.legend(loc="lower right", fontsize=9)
        self.fig.canvas.draw()

        # Convert canvas to RGB array
        buf = self.fig.canvas.buffer_rgba()
        ncols, nrows = self.fig.canvas.get_width_height()
        img = np.frombuffer(buf, dtype=np.uint8).reshape(nrows, ncols, 4)
        return cv2.cvtColor(img, cv2.COLOR_RGBA2RGB) if _HAS_CV2 else img[:, :, :3]

    @staticmethod
    def _draw_covariance_ellipse(
        mean: np.ndarray,
        cov: np.ndarray,
        n_std: float = 2.0,
        **kwargs: Any
    ) -> Any:
        """Draw a covariance ellipse on the given axes.

        Args:
            mean: Centre of ellipse (2,).
            cov: 2x2 covariance matrix.
            n_std: Number of standard deviations for ellipse radius.
            **kwargs: Passed to ``matplotlib.patches.Ellipse``.

        Returns:
            The Ellipse artist.
        """
        vals, vecs = np.linalg.eigh(cov)
        order = vals.argsort()[::-1]
        vals = vals[order]
        vecs = vecs[:, order]

        angle = np.degrees(np.arctan2(*vecs[:, 0][::-1]))
        width, height = 2 * n_std * np.sqrt(vals)

        ellipse = matplotlib.patches.Ellipse(
            xy=mean, width=width, height=height, angle=angle,
            edgecolor="red", facecolor="none", linewidth=2, **kwargs
        )
        plt.gca().add_patch(ellipse)
        return ellipse


# ---------------------------------------------------------------------------
# Self-test
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import sys

    logging.basicConfig(
        level=logging.DEBUG,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s"
    )

    print("=" * 60)
    print("pipeline.py self-test")
    print("=" * 60)

    # 1. _MinimalEKFStub
    ekf = _MinimalEKFStub(process_noise=0.1, measurement_noise_wifi=2.0, measurement_noise_cam=0.3)
    pos, cov = ekf.get_position()
    assert pos.shape == (2,), f"Expected pos shape (2,), got {pos.shape}"
    assert cov.shape == (2, 2), f"Expected cov shape (2,2), got {cov.shape}"
    print("[PASS] _MinimalEKFStub init + get_position")

    # 2. _MinimalEKFStub predict
    ekf.predict(dt=0.1)
    pos2, _ = ekf.get_position()
    assert np.allclose(pos, pos2), "Position should not change with zero velocity"
    print("[PASS] _MinimalEKFStub predict")

    # 3. _MinimalEKFStub WiFi update
    ekf.update_wifi(np.array([3.0, 2.0]))
    pos3, cov3 = ekf.get_position()
    assert np.linalg.norm(pos3 - np.array([3.0, 2.0])) < 2.0, "Update should shift position toward measurement"
    print("[PASS] _MinimalEKFStub update_wifi")

    # 4. _MinimalEKFStub camera update
    ekf.update_camera(np.array([3.2, 2.1]))
    pos4, _ = ekf.get_position()
    assert pos4 is not None
    print("[PASS] _MinimalEKFStub update_camera")

    # 5. _MinimalEKFStub bias estimation
    bias = ekf.estimate_wifi_bias(np.array([3.0, 2.0]), np.array([2.8, 1.9]))
    assert np.allclose(bias, np.array([0.2, 0.1]))
    print("[PASS] _MinimalEKFStub estimate_wifi_bias")

    # 6. _CSISourceStub
    stub = _CSISourceStub("192.168.1.100", 5005)
    stub.start()
    w = stub.get_window(10)
    assert w is not None and w.shape[0] == 10
    stub.stop()
    print("[PASS] _CSISourceStub")

    # 7. TrackingPipeline instantiation with dummy configs
    class _DummyCSIConfig:
        def __init__(self):
            self.esp32_ip = ["192.168.1.100"]
            self.esp32_port = 5005
            self.use_multi_esp = False
            self.window_size_ms = 100

    class _DummyYOLOConfig:
        def __init__(self):
            self.model = "yolo11n.pt"
            self.conf_thresh = 0.5
            self.camera_id = 0
            self.resolution = (640, 480)

    class _DummyFusionConfig:
        def __init__(self):
            self.process_noise = 0.1
            self.measurement_noise_wifi = 2.0
            self.measurement_noise_cam = 0.3
            self.fusion_weight_wifi = 0.3
            self.fusion_weight_cam = 0.7

    csi_cfg = _DummyCSIConfig()
    yolo_cfg = _DummyYOLOConfig()
    fusion_cfg = _DummyFusionConfig()

    pipeline = TrackingPipeline(csi_cfg, yolo_cfg, fusion_cfg)
    assert not pipeline.running
    assert pipeline.ekf is not None
    assert "csi" in pipeline.queues
    assert "yolo" in pipeline.queues
    assert "fusion" in pipeline.queues
    print("[PASS] TrackingPipeline instantiation")

    # 8. get_state() before start
    state = pipeline.get_state()
    assert "position" in state
    assert "covariance" in state
    assert "wifi_bias" in state
    assert state["position"] is not None
    print("[PASS] TrackingPipeline get_state (pre-start)")

    # 9. PipelineVisualizer (if matplotlib available)
    if _HAS_MPL:
        vis = PipelineVisualizer(
            room_bounds=(0, 5, 0, 4),
            anchor_positions=np.array([[0, 0], [5, 0], [5, 4]])
        )
        vis.update(state)
        img = vis.render()
        assert img.ndim == 3 and img.shape[2] == 3
        assert img.dtype == np.uint8
        print(f"[PASS] PipelineVisualizer render ({img.shape})")
    else:
        print("[SKIP] PipelineVisualizer (matplotlib not installed)")

    # 10. Queue back-pressure
    for i in range(110):
        try:
            pipeline.queues["csi"].put_nowait({"idx": i})
        except Exception:
            pass
    assert pipeline.queues["csi"].qsize() <= 100
    print("[PASS] Queue maxsize back-pressure")

    print("=" * 60)
    print("Self-test complete.")
    print("=" * 60)
