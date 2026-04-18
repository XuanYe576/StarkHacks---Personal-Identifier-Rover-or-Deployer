"""
Sensor Fusion: EKF-based WiFi + Camera Position Tracking

Extended Kalman Filter with adaptive covariance, camera-assisted
phase drift correction, and sensor dropout handling.
"""

import logging
import time
from typing import Optional, Tuple

import numpy as np

logger = logging.getLogger(__name__)

# --------------------------------------------------------------------------- #
# Adaptive weight assignment
# --------------------------------------------------------------------------- #


def adaptive_weight_assignment(
    wifi_variance: float,
    cam_variance: float,
    wifi_snr: float,
    cam_confidence: float,
) -> Tuple[float, float]:
    """Compute dynamic sensor weights that sum to 1.0.

    The weighting strategy combines measurement quality (variance) and
    auxiliary confidence metrics (SNR / detection confidence).  Lower
    variance and higher confidence produce a larger weight.

    Parameters
    ----------
    wifi_variance:
        Estimated variance of the WiFi position measurement.
    cam_variance:
        Estimated variance of the camera position measurement.
    wifi_snr:
        WiFi signal-to-noise ratio in dB (larger is better).
    cam_confidence:
        YOLO detection confidence in [0, 1] (larger is better).

    Returns
    -------
    Tuple[float, float]
        ``(w_wifi, w_cam)`` with ``w_wifi + w_cam == 1.0``.
    """
    # Information = 1 / variance
    info_wifi = 1.0 / (wifi_variance + 1e-6)
    info_cam = 1.0 / (cam_variance + 1e-6)

    # Convert auxiliary metrics to reliability factors
    # SNR factor: sigmoid-like mapping from dB → [0, 1]
    wifi_reliability = 1.0 / (1.0 + np.exp(-0.3 * (wifi_snr - 10.0)))
    # Camera confidence is already in [0, 1]
    cam_reliability = float(np.clip(cam_confidence, 0.0, 1.0))

    # Combined score
    score_wifi = info_wifi * (0.5 + 0.5 * wifi_reliability)
    score_cam = info_cam * (0.5 + 0.5 * cam_reliability)

    total = score_wifi + score_cam + 1e-9
    w_wifi = score_wifi / total
    w_cam = score_cam / total

    return float(w_wifi), float(w_cam)


# --------------------------------------------------------------------------- #
# Fusion EKF
# --------------------------------------------------------------------------- #


class CSI_YOLO_FusionEKF:
    """EKF fusing WiFi CSI localization with YOLO camera tracking.

    State vector: ``[px, py, vx, vy]^T``

    The filter uses a constant-velocity motion model and accepts 2-D
    position updates from either (or both) WiFi and camera sensors.
    When both sensors are active the filter also estimates and
    compensates a slowly-varying WiFi measurement bias, enabling
    camera-assisted phase drift correction.

    Parameters
    ----------
    Q : float
        Process noise scale factor (applied to identity).
    R_wifi : float
        WiFi measurement noise scale factor.
    R_cam : float
        Camera measurement noise scale factor.
    """

    def __init__(
        self,
        Q: float = 0.1,
        R_wifi: float = 2.0,
        R_cam: float = 0.3,
    ):
        # State: [px, py, vx, vy]
        self.x: np.ndarray = np.zeros(4)
        # Initial covariance
        self.P: np.ndarray = np.eye(4) * 10.0
        # Process noise
        self.Q: np.ndarray = np.eye(4) * float(Q)
        # WiFi measurement noise
        self.R_wifi: np.ndarray = np.eye(2) * float(R_wifi)
        # Camera measurement noise
        self.R_cam: np.ndarray = np.eye(2) * float(R_cam)

        # WiFi bias estimate (2-D offset)
        self.wifi_bias: np.ndarray = np.zeros(2)

        # Timestamps for sensor dropout detection
        self.last_wifi_update: float = 0.0
        self.last_cam_update: float = 0.0

        # Statistics
        self._n_wifi_updates: int = 0
        self._n_cam_updates: int = 0

    # ------------------------------------------------------------------ #
    # Prediction
    # ------------------------------------------------------------------ #

    def predict(self, dt: float = 0.1) -> None:
        """Constant-velocity prediction step.

        Parameters
        ----------
        dt:
            Time step in seconds.
        """
        dt = float(dt)
        F = np.array(
            [[1.0, 0.0, dt, 0.0],
             [0.0, 1.0, 0.0, dt],
             [0.0, 0.0, 1.0, 0.0],
             [0.0, 0.0, 0.0, 1.0]],
            dtype=float,
        )
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q

        # Inflate covariance if a sensor has dropped out
        now = time.time()
        if now - self.last_wifi_update > 2.0 and now - self.last_cam_update > 2.0:
            # Both sensors silent — inflate position uncertainty
            self.P[:2, :2] += np.eye(2) * 5.0
            logger.debug("both sensors stale — inflating covariance")

    # ------------------------------------------------------------------ #
    # WiFi update
    # ------------------------------------------------------------------ #

    def update_wifi(
        self, pos: np.ndarray, variance: Optional[float] = None
    ) -> None:
        """Update the filter with a WiFi-derived position estimate.

        The WiFi measurement is bias-compensated before the update so
        that drift estimated from camera cross-calibration does not
        accumulate.

        Parameters
        ----------
        pos:
            Measured (x, y) position, shape (2,).
        variance:
            Optional scalar variance.  When ``None`` the default
            ``R_wifi`` matrix is used; otherwise ``R_wifi * variance``.
        """
        pos = np.asarray(pos, dtype=float).ravel()
        if pos.shape[0] != 2:
            raise ValueError("pos must be a 2-element vector")

        H = np.array([[1.0, 0.0, 0.0, 0.0],
                      [0.0, 1.0, 0.0, 0.0]], dtype=float)

        if variance is not None:
            R = self.R_wifi * float(variance)
        else:
            R = self.R_wifi.copy()

        # Compensate estimated bias
        pos_corrected = pos - self.wifi_bias
        self._update(H, pos_corrected, R)
        self.last_wifi_update = time.time()
        self._n_wifi_updates += 1

    # ------------------------------------------------------------------ #
    # Camera update
    # ------------------------------------------------------------------ #

    def update_camera(
        self, pos: np.ndarray, variance: Optional[float] = None
    ) -> None:
        """Update the filter with a camera-derived position estimate.

        When a recent WiFi update exists (within the last second) the
        difference between the camera and the current filtered position
        is used to refine the WiFi bias estimate.  This implements the
        *camera-assisted phase drift correction* feature.

        Parameters
        ----------
        pos:
            Measured (x, y) position, shape (2,).
        variance:
            Optional scalar variance.  When ``None`` the default
            ``R_cam`` matrix is used; otherwise ``R_cam * variance``.
        """
        pos = np.asarray(pos, dtype=float).ravel()
        if pos.shape[0] != 2:
            raise ValueError("pos must be a 2-element vector")

        H = np.array([[1.0, 0.0, 0.0, 0.0],
                      [0.0, 1.0, 0.0, 0.0]], dtype=float)

        if variance is not None:
            R = self.R_cam * float(variance)
        else:
            R = self.R_cam.copy()

        self._update(H, pos, R)

        # Update WiFi bias estimate when both sensors are available
        now = time.time()
        if now - self.last_wifi_update < 1.0:
            # Bias = WiFi_raw - true_position ≈ WiFi_raw - camera_pos
            # Use exponential moving average for smooth bias tracking
            alpha = 0.1
            raw_wifi_pos = self.x[:2] + self.wifi_bias
            new_bias = raw_wifi_pos - pos
            self.wifi_bias = (1.0 - alpha) * self.wifi_bias + alpha * new_bias
            logger.debug("wifi bias updated: %s", self.wifi_bias)

        self.last_cam_update = now
        self._n_cam_updates += 1

    # ------------------------------------------------------------------ #
    # Generic EKF update
    # ------------------------------------------------------------------ #

    def _update(self, H: np.ndarray, z: np.ndarray, R: np.ndarray) -> None:
        """Generic Kalman update step.

        Parameters
        ----------
        H:
            Observation matrix, shape (m, 4).
        z:
            Measurement vector, shape (m,).
        R:
            Measurement noise covariance, shape (m, m).
        """
        y = z - H @ self.x                         # innovation
        S = H @ self.P @ H.T + R                    # innovation covariance
        try:
            K = self.P @ H.T @ np.linalg.inv(S)     # Kalman gain
        except np.linalg.LinAlgError:
            logger.warning("singular innovation covariance — skipping update")
            return
        self.x = self.x + K @ y
        I_KH = np.eye(4) - K @ H
        # Joseph form for numerical stability
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T

    # ------------------------------------------------------------------ #
    # Accessors
    # ------------------------------------------------------------------ #

    def get_position(self) -> Tuple[np.ndarray, np.ndarray]:
        """Return the current position estimate and covariance.

        Returns
        -------
        Tuple[np.ndarray, np.ndarray]
            ``(position, covariance)`` where *position* has shape (2,)
            and *covariance* has shape (2, 2).
        """
        return self.x[:2].copy(), self.P[:2, :2].copy()

    def get_velocity(self) -> Tuple[np.ndarray, np.ndarray]:
        """Return the current velocity estimate and covariance.

        Returns
        -------
        Tuple[np.ndarray, np.ndarray]
            ``(velocity, velocity_covariance)`` each with shape (2,).
        """
        return self.x[2:].copy(), self.P[2:, 2:].copy()

    def estimate_wifi_bias(
        self, camera_pos: np.ndarray, wifi_pos: np.ndarray
    ) -> np.ndarray:
        """Compute the WiFi measurement bias relative to camera ground truth.

        This is the *instantaneous* bias between a raw WiFi position and
        the (more accurate) camera position.  For online bias tracking
        the filter uses an exponential moving average inside
        :meth:`update_camera`.

        Parameters
        ----------
        camera_pos:
            Camera-derived position, shape (2,).
        wifi_pos:
            Raw WiFi-derived position, shape (2,).

        Returns
        -------
        np.ndarray
            Bias vector ``wifi_pos - camera_pos``, shape (2,).
        """
        return np.asarray(wifi_pos, dtype=float) - np.asarray(
            camera_pos, dtype=float
        )

    def get_status(self) -> dict:
        """Return diagnostic status dictionary."""
        pos, cov = self.get_position()
        return {
            "position": pos.tolist(),
            "covariance_trace": float(np.trace(self.P)),
            "wifi_bias": self.wifi_bias.tolist(),
            "n_wifi_updates": self._n_wifi_updates,
            "n_cam_updates": self._n_cam_updates,
            "wifi_stale_s": time.time() - self.last_wifi_update,
            "cam_stale_s": time.time() - self.last_cam_update,
        }


# --------------------------------------------------------------------------- #
# Self-test
# --------------------------------------------------------------------------- #


def _test() -> None:
    """Run basic correctness checks on the fusion filter."""
    logging.basicConfig(level=logging.DEBUG)
    print("=" * 60)
    print("SENSOR FUSION MODULE SELF-TEST")
    print("=" * 60)

    # --- adaptive_weight_assignment ---
    print("\n--- adaptive_weight_assignment ---")
    w_wifi, w_cam = adaptive_weight_assignment(
        wifi_variance=2.0, cam_variance=0.3, wifi_snr=15.0, cam_confidence=0.85
    )
    print(f"  w_wifi={w_wifi:.4f}  w_cam={w_cam:.4f}  sum={w_wifi + w_cam:.4f}")
    assert abs(w_wifi + w_cam - 1.0) < 1e-6
    # Camera should dominate (lower variance + high confidence)
    assert w_cam > w_wifi

    # Bad WiFi → camera should dominate even more
    w_wifi2, w_cam2 = adaptive_weight_assignment(
        wifi_variance=10.0, cam_variance=0.3, wifi_snr=5.0, cam_confidence=0.9
    )
    print(f"  bad wifi: w_wifi={w_wifi2:.4f}  w_cam={w_cam2:.4f}")
    assert w_cam2 > w_wifi2

    # --- CSI_YOLO_FusionEKF basic lifecycle ---
    print("\n--- FusionEKF ---")
    ekf = CSI_YOLO_FusionEKF(Q=0.1, R_wifi=2.0, R_cam=0.3)
    pos0, cov0 = ekf.get_position()
    print(f"  init pos={pos0}, cov trace={np.trace(cov0):.4f}")
    assert np.allclose(pos0, 0.0)

    # Predict
    ekf.predict(dt=0.1)
    pos1, cov1 = ekf.get_position()
    print(f"  after predict: pos={pos1}, cov trace={np.trace(cov1):.4f}")

    # WiFi-only update
    ekf.update_wifi(np.array([5.0, 3.0]))
    pos2, cov2 = ekf.get_position()
    print(f"  after WiFi:    pos={pos2}, cov trace={np.trace(cov2):.4f}")
    # Position should move toward the WiFi measurement
    assert np.linalg.norm(pos2 - np.array([5.0, 3.0])) < np.linalg.norm(
        pos1 - np.array([5.0, 3.0])
    )

    # Predict + camera update
    ekf.predict(dt=0.1)
    ekf.update_camera(np.array([4.5, 3.2]))
    pos3, cov3 = ekf.get_position()
    print(f"  after Camera:  pos={pos3}, cov trace={np.trace(cov3):.4f}")
    # Camera has lower noise → position should be closer to camera
    dist_to_cam = np.linalg.norm(pos3 - np.array([4.5, 3.2]))
    dist_to_wifi = np.linalg.norm(pos3 - np.array([5.0, 3.0]))
    assert dist_to_cam < dist_to_wifi, "camera should pull state more"

    # --- WiFi bias estimation ---
    print("\n--- WiFi bias estimation ---")
    bias = ekf.estimate_wifi_bias(
        camera_pos=np.array([4.5, 3.2]), wifi_pos=np.array([5.0, 3.0])
    )
    print(f"  instantaneous bias = {bias}")
    assert np.allclose(bias, np.array([0.5, -0.2]))

    # Test bias tracking via update_camera
    ekf2 = CSI_YOLO_FusionEKF(Q=0.1, R_wifi=2.0, R_cam=0.3)
    # Simulate WiFi consistently offset by [1, -0.5]
    # Run more iterations to allow EMA bias estimate to converge
    true_bias = np.array([1.0, -0.5])
    for i in range(120):
        ekf2.predict(dt=0.1)
        true_pos = np.array([2.0 + 0.01 * i, 1.0])
        wifi_meas = true_pos + true_bias                # biased WiFi
        ekf2.update_wifi(wifi_meas)
        ekf2.predict(dt=0.1)
        ekf2.update_camera(true_pos)                    # accurate camera

    bias_est = ekf2.wifi_bias
    print(f"  tracked bias (should be ~[1, -0.5]): {bias_est}")
    assert np.linalg.norm(bias_est - true_bias) < 0.4, \
        f"bias {bias_est} did not converge toward {true_bias}"

    # --- Variance scaling ---
    print("\n--- variance scaling ---")
    ekf3 = CSI_YOLO_FusionEKF(Q=0.1, R_wifi=2.0, R_cam=0.3)
    ekf3.update_wifi(np.array([10.0, 10.0]), variance=0.01)
    pos_high_conf, _ = ekf3.get_position()
    print(f"  high-confidence WiFi: pos={pos_high_conf}")
    # With low variance the update should pull the state strongly
    assert np.linalg.norm(pos_high_conf - np.array([10.0, 10.0])) < 2.0