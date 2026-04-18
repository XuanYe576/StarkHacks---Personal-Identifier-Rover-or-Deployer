"""
Multi-ESP32 FFT Processing & Phase Smoothing

Handles 100ms Savitzky-Golay phase smoothing, FFT-based channel analysis,
phase slope estimation for ToF, and distance calculation per ESP32.

Designed for unsynchronized ESP32s where phase incoherence is mitigated
via 100ms moving-window smoothing (Savitzky-Golay filter).
"""

import logging
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Dict, List, Optional, Tuple

import numpy as np
from scipy.signal import savgol_filter

logger = logging.getLogger(__name__)

# Speed of light (m/s)
_C = 3e8


# ---------------------------------------------------------------------------
# Phase Smoothing
# ---------------------------------------------------------------------------

def phase_smooth_savgol(
    phase: np.ndarray,
    window_ms: int = 100,
    fs: int = 100,
    polyorder: int = 3,
) -> np.ndarray:
    """Smooth phase using a Savitzky-Golay filter over a time window.

    Converts the time-domain window to samples and applies a polynomial
    least-squares filter. For 2-D inputs (samples x subcarriers), each
    subcarrier is smoothed independently.

    Args:
        phase: Phase array. Can be 1-D (n_samples,) or 2-D
            (n_samples, n_subcarriers) in radians.
        window_ms: Smoothing window duration in milliseconds. Default 100.
        fs: Sampling rate in Hz. Default 100.
        polyorder: Polynomial order for the Savitzky-Golay filter.
            Default 3. Must be less than window length.

    Returns:
        Smoothed phase with the same shape as input.

    Raises:
        ValueError: If polyorder >= window length or phase is empty.
    """
    phase = np.asarray(phase)
    if phase.size == 0:
        raise ValueError("Phase array is empty")

    # Convert window duration to samples
    window_len = int(window_ms * fs / 1000)
    if window_len < 3:
        logger.warning(
            "Window length %d too small (fs=%d, window_ms=%d); returning input",
            window_len, fs, window_ms,
        )
        return phase
    if window_len % 2 == 0:
        window_len += 1  # Must be odd

    if polyorder >= window_len:
        polyorder = window_len - 2
        if polyorder < 1:
            polyorder = 1
        logger.debug("Reduced polyorder to %d to fit window length %d", polyorder, window_len)

    if phase.ndim == 1:
        if phase.size < window_len:
            logger.debug(
                "Phase length %d < window %d; returning input", phase.size, window_len
            )
            return phase
        return savgol_filter(phase, window_length=window_len, polyorder=polyorder)
    elif phase.ndim == 2:
        n_samples, n_subcarriers = phase.shape
        if n_samples < window_len:
            logger.debug(
                "Phase samples %d < window %d; returning input", n_samples, window_len
            )
            return phase
        smoothed = np.zeros_like(phase)
        for k in range(n_subcarriers):
            smoothed[:, k] = savgol_filter(
                phase[:, k], window_length=window_len, polyorder=polyorder
            )
        return smoothed
    else:
        raise ValueError(f"phase must be 1-D or 2-D, got {phase.ndim}D")


# ---------------------------------------------------------------------------
# Phase Slope & ToF Estimation
# ---------------------------------------------------------------------------

def estimate_phase_slope(
    phase: np.ndarray,
    subcarrier_frequencies: np.ndarray,
) -> float:
    """Estimate the phase slope via linear regression.

    Models phase as a linear function of subcarrier frequency:
        phi_k = slope * f_k + intercept

    The slope is proportional to the time-of-flight:
        slope = -2 * pi * tau  (rad/Hz)

    Args:
        phase: 1-D array of phase values in radians, one per subcarrier.
        subcarrier_frequencies: 1-D array of subcarrier frequencies in Hz.

    Returns:
        slope: Phase slope in rad/Hz.

    Raises:
        ValueError: If inputs are not 1-D or have mismatched lengths.
    """
    phase = np.asarray(phase).flatten()
    freqs = np.asarray(subcarrier_frequencies).flatten()

    if phase.ndim != 1 or freqs.ndim != 1:
        raise ValueError("Both inputs must be 1-D arrays")
    if len(phase) != len(freqs):
        raise ValueError(
            f"Phase length ({len(phase)}) must match frequencies length ({len(freqs)})"
        )
    if len(phase) < 2:
        logger.warning("Need at least 2 points for slope estimation; returning 0")
        return 0.0

    # Linear fit: phase = slope * f + intercept
    coeffs = np.polyfit(freqs, phase, 1)
    slope = float(coeffs[0])
    return slope


def csi_to_tof(csi_frame: np.ndarray, bw_hz: float = 20e6) -> float:
    """Convert a single CSI frame to time-of-flight estimate.

    Extracts phase from complex CSI values, generates subcarrier frequency
    offsets, and estimates the phase slope. The ToF is derived from the
    phase-frequency slope:
        tau = -slope / (2 * pi)

    Args:
        csi_frame: Complex-valued 1-D array of CSI subcarrier values.
        bw_hz: Channel bandwidth in Hz. Default 20 MHz (HT20).

    Returns:
        tof: Estimated time-of-flight in seconds.

    Raises:
        ValueError: If csi_frame is not 1-D.
    """
    csi_frame = np.asarray(csi_frame)
    if csi_frame.ndim != 1:
        raise ValueError(f"csi_frame must be 1-D, got {csi_frame.ndim}D")
    n_subcarriers = len(csi_frame)
    if n_subcarriers == 0:
        logger.warning("Empty CSI frame; returning ToF=0")
        return 0.0

    # Extract phase
    phase = np.angle(csi_frame)

    # Subcarrier frequency offsets: f_k = k * delta_f
    # Centered around DC (0 Hz)
    delta_f = bw_hz / n_subcarriers
    freqs = (np.arange(n_subcarriers) - (n_subcarriers - 1) / 2.0) * delta_f

    slope = estimate_phase_slope(phase, freqs)
    tof = -slope / (2.0 * np.pi)
    return tof


def csi_to_distance(csi_frame: np.ndarray, bw_hz: float = 20e6) -> float:
    """Convert CSI frame to distance estimate.

    Uses the round-trip time-of-flight relationship:
        d = c * tau / 2

    where c is the speed of light and tau is the one-way ToF (half of
    round-trip).

    Args:
        csi_frame: Complex-valued 1-D array of CSI subcarrier values.
        bw_hz: Channel bandwidth in Hz. Default 20 MHz.

    Returns:
        distance: Estimated distance in meters, clamped to [0.1, 50].
    """
    tof = csi_to_tof(csi_frame, bw_hz)
    distance = _C * tof / 2.0  # round-trip

    # Clamp to physically reasonable range
    distance = float(np.clip(distance, 0.1, 50.0))
    return distance


# ---------------------------------------------------------------------------
# FFT-based CSI Analysis
# ---------------------------------------------------------------------------

def fft_smooth_csi(
    csi_frames: List[np.ndarray],
    n_fft: int = 256,
) -> np.ndarray:
    """Apply FFT-based frequency analysis to a series of CSI frames.

    Stacks CSI frames into a time-series matrix and computes the FFT along
    the time axis. This reveals frequency components in the channel state
    dynamics (e.g., breathing at ~0.1-0.5 Hz, movement at higher freq).

    Args:
        csi_frames: List of N complex-valued 1-D or 2-D CSI frames.
            Each frame should have shape (n_subcarriers,).
        n_fft: Number of FFT points. Default 256.

    Returns:
        magnitude_spectrum: 2-D array of shape (n_subcarriers, n_fft//2+1)
            containing the magnitude spectrum per subcarrier.

    Raises:
        ValueError: If csi_frames is empty or frames have inconsistent shapes.
    """
    if not csi_frames:
        raise ValueError("csi_frames list is empty")

    # Stack into time-series matrix: (n_subcarriers, n_frames)
    stacked = np.column_stack([np.asarray(f).flatten() for f in csi_frames])
    n_subcarriers, n_frames = stacked.shape

    if n_frames < 2:
        logger.warning("Need >= 2 frames for FFT; returning zeros")
        return np.zeros((n_subcarriers, n_fft // 2 + 1))

    # Zero-pad or truncate along time axis
    if n_frames < n_fft:
        pad_width = n_fft - n_frames
        stacked = np.pad(stacked, ((0, 0), (0, pad_width)), mode="constant")
    else:
        stacked = stacked[:, :n_fft]

    # FFT along time axis (axis=1)
    # Use np.fft.fft for complex input, then keep positive frequencies
    spectrum_full = np.fft.fft(stacked, axis=1)  # shape (n_subcarriers, n_fft)
    n_pos = n_fft // 2 + 1
    spectrum = spectrum_full[:, :n_pos]
    magnitude_spectrum = np.abs(spectrum)
    return magnitude_spectrum


# ---------------------------------------------------------------------------
# Time Alignment & CFO Compensation
# ---------------------------------------------------------------------------

def align_esp_timestamps(
    esp_streams: Dict[str, List[Dict]],
) -> Dict[str, List[Dict]]:
    """Align multiple ESP32 streams to a common time base.

    Finds the overlapping time window across all streams and uses
    nearest-neighbor matching to align samples.

    Args:
        esp_streams: Dictionary mapping ESP ID to a list of frame dicts.
            Each frame dict must contain a 'timestamp' key (float, seconds).

    Returns:
        Aligned streams with the same structure, where each stream
        contains only frames within the common time window.

    Raises:
        ValueError: If esp_streams is empty or any stream has no timestamps.
    """
    if not esp_streams:
        return {}

    # Extract timestamps per stream
    timestamps = {}
    for esp_id, frames in esp_streams.items():
        if not frames:
            logger.warning("Stream %s is empty", esp_id)
            continue
        ts = [float(f.get("timestamp", 0.0)) for f in frames]
        timestamps[esp_id] = np.asarray(ts)

    if not timestamps:
        logger.warning("No valid timestamps found")
        return esp_streams

    # Find common time window
    t_starts = [np.min(ts) for ts in timestamps.values()]
    t_ends = [np.max(ts) for ts in timestamps.values()]
    t_common_start = np.max(t_starts)
    t_common_end = np.min(t_ends)

    if t_common_start >= t_common_end:
        logger.warning(
            "No overlapping time window (start=%.3f, end=%.3f)",
            t_common_start, t_common_end,
        )
        return esp_streams

    # Use the densest stream as reference
    ref_id = max(timestamps, key=lambda k: len(timestamps[k]))
    ref_ts = timestamps[ref_id]
    ref_mask = (ref_ts >= t_common_start) & (ref_ts <= t_common_end)
    ref_ts_aligned = ref_ts[ref_mask]

    if len(ref_ts_aligned) == 0:
        logger.warning("Reference stream empty after clipping")
        return esp_streams

    aligned_streams: Dict[str, List[Dict]] = {}
    for esp_id, frames in esp_streams.items():
        ts = timestamps[esp_id]
        # Find indices in common window
        mask = (ts >= t_common_start) & (ts <= t_common_end)
        filtered_frames = [f for f, m in zip(frames, mask) if m]
        aligned_streams[esp_id] = filtered_frames

    logger.debug(
        "Aligned %d streams to common window [%.3f, %.3f] (%d reference points)",
        len(aligned_streams), t_common_start, t_common_end, len(ref_ts_aligned),
    )
    return aligned_streams


def compensate_cfo_drift(
    phase_series: np.ndarray,
    timestamps: np.ndarray,
) -> np.ndarray:
    """Estimate and remove linear CFO drift from a phase time series.

    Uses linear regression to model the drift:
        phase_drift(t) = a * t + b
    
    The linear trend is subtracted from the phase series.

    Args:
        phase_series: 1-D or 2-D array of phase values in radians.
            If 2-D, shape is (n_samples, n_subcarriers).
        timestamps: 1-D array of timestamps in seconds, same length as
            the first dimension of phase_series.

    Returns:
        Compensated phase series with the same shape as input.

    Raises:
        ValueError: If inputs have incompatible shapes.
    """
    phase_series = np.asarray(phase_series)
    timestamps = np.asarray(timestamps).flatten()

    if phase_series.size == 0 or timestamps.size == 0:
        raise ValueError("Empty input")

    n_samples = phase_series.shape[0]
    if len(timestamps) != n_samples:
        raise ValueError(
            f"Timestamp count ({len(timestamps)}) must match phase samples ({n_samples})"
        )
    if n_samples < 2:
        logger.debug("Need >= 2 samples for drift compensation; returning input")
        return phase_series

    # Normalize timestamps for numerical stability
    t_centered = timestamps - np.mean(timestamps)

    if phase_series.ndim == 1:
        coeffs = np.polyfit(t_centered, phase_series, 1)
        drift = np.polyval(coeffs, t_centered)
        return phase_series - drift
    elif phase_series.ndim == 2:
        compensated = np.zeros_like(phase_series)
        for k in range(phase_series.shape[1]):
            coeffs = np.polyfit(t_centered, phase_series[:, k], 1)
            drift = np.polyval(coeffs, t_centered)
            compensated[:, k] = phase_series[:, k] - drift
        return compensated
    else:
        raise ValueError(f"phase_series must be 1-D or 2-D, got {phase_series.ndim}D")


# ---------------------------------------------------------------------------
# MultiESPProcessor
# ---------------------------------------------------------------------------

@dataclass
class ESPConfig:
    """Configuration for a single ESP32 device."""
    esp_id: str
    ip: str = ""
    port: int = 5005
    position: Optional[np.ndarray] = None  # (x, y) in meters


class MultiESPProcessor:
    """Orchestrates multi-ESP32 CSI processing with 100ms phase smoothing.

    Maintains per-ESP circular buffers, applies Savitzky-Golay smoothing,
    computes distance estimates from phase slope, and tracks synchronization
    status across all ESP streams.

    Attributes:
        esp_configs: List of ESPConfig objects.
        window_ms: Phase smoothing window in milliseconds.
        fs: Target sample rate in Hz.
    """

    def __init__(
        self,
        esp_configs: List[ESPConfig],
        window_ms: int = 100,
        fs: int = 100,
    ):
        """Initialize the multi-ESP processor.

        Args:
            esp_configs: List of ESPConfig instances.
            window_ms: Phase smoothing window in milliseconds. Default 100.
            fs: Target sample rate in Hz. Default 100.
        """
        self.esp_configs = esp_configs
        self.window_ms = window_ms
        self.fs = fs

        # Per-ESP circular buffers for raw CSI frames
        self._buffers: Dict[str, Deque[Dict]] = {
            cfg.esp_id: deque(maxlen=int(fs * window_ms / 1000) * 5)
            for cfg in esp_configs
        }

        # Per-ESP smoothed phase history
        self._phase_history: Dict[str, Deque[np.ndarray]] = {
            cfg.esp_id: deque(maxlen=int(fs * window_ms / 1000) * 5)
            for cfg in esp_configs
        }

        # Per-ESP distance estimates
        self._distances: Dict[str, float] = {
            cfg.esp_id: float("nan") for cfg in esp_configs
        }

        # Timing statistics per ESP
        self._last_timestamp: Dict[str, float] = {
            cfg.esp_id: 0.0 for cfg in esp_configs
        }
        self._inter_arrival: Dict[str, Deque[float]] = {
            cfg.esp_id: deque(maxlen=100) for cfg in esp_configs
        }
        self._frame_counts: Dict[str, int] = {
            cfg.esp_id: 0 for cfg in esp_configs
        }

        logger.info(
            "MultiESPProcessor initialized with %d ESP(s), window_ms=%d, fs=%d",
            len(esp_configs), window_ms, fs,
        )

    def process_frame(self, esp_id: str, frame: Dict) -> Optional[Dict]:
        """Process a single CSI frame from an ESP32.

        Adds the frame to the per-ESP buffer and, if the buffer has
        enough samples, applies phase smoothing and returns the
        processed result.

        Args:
            esp_id: Identifier for the ESP32 device.
            frame: Dictionary containing at minimum:
                - 'timestamp': float (seconds)
                - 'csi': complex np.ndarray (subcarrier values)

        Returns:
            Dictionary with smoothed output if the window is full,
            otherwise None.

        Raises:
            ValueError: If esp_id is not in the configuration.
        """
        if esp_id not in self._buffers:
            raise ValueError(f"Unknown ESP ID: {esp_id}")

        ts = float(frame.get("timestamp", 0.0))
        csi = np.asarray(frame.get("csi", np.array([])))

        if csi.size == 0:
            logger.warning("Empty CSI from %s", esp_id)
            return None

        # Track inter-arrival time
        if self._frame_counts[esp_id] > 0:
            dt = ts - self._last_timestamp[esp_id]
            if dt > 0:
                self._inter_arrival[esp_id].append(dt)
        self._last_timestamp[esp_id] = ts
        self._frame_counts[esp_id] += 1

        # Store frame
        self._buffers[esp_id].append({"timestamp": ts, "csi": csi})

        # Store phase
        phase = np.angle(csi)
        self._phase_history[esp_id].append(phase)

        # Check if we have enough samples for smoothing
        window_len = int(self.window_ms * self.fs / 1000)
        if window_len % 2 == 0:
            window_len += 1

        buf = self._buffers[esp_id]
        if len(buf) >= window_len:
            # Extract time-series of phases
            phases = np.array([np.angle(e["csi"]) for e in buf])
            timestamps = np.array([e["timestamp"] for e in buf])

            # Apply Savitzky-Golay smoothing
            smoothed_phase = phase_smooth_savgol(
                phases, window_ms=self.window_ms, fs=self.fs
            )

            # Compensate CFO drift
            compensated_phase = compensate_cfo_drift(smoothed_phase, timestamps)

            # Use latest smoothed phase for distance estimation
            latest_phase = compensated_phase[-1, :]
            latest_csi = np.abs(buf[-1]["csi"]) * np.exp(1j * latest_phase)

            # Update distance
            dist = csi_to_distance(latest_csi)
            self._distances[esp_id] = dist

            # Build smoothed frame
            smoothed_frame = {
                "esp_id": esp_id,
                "timestamp": ts,
                "csi": latest_csi,
                "phase": latest_phase,
                "distance_m": dist,
                "buffer_size": len(buf),
            }
            return smoothed_frame

        return None

    def get_smoothed_distances(self) -> Dict[str, float]:
        """Return the latest smoothed distance estimate for each ESP.

        Returns:
            Dictionary mapping ESP ID to distance in meters.
            NaN indicates no valid estimate yet.
        """
        return dict(self._distances)

    def get_sync_status(self) -> Dict:
        """Return synchronization and timing statistics per ESP.

        Returns:
            Dictionary with keys:
                - 'per_esp': dict of per-ESP stats
                - 'overall': overall system stats
        """
        per_esp = {}
        for cfg in self.esp_configs:
            eid = cfg.esp_id
            iats = list(self._inter_arrival[eid])
            per_esp[eid] = {
                "frame_count": self._frame_counts[eid],
                "buffer_size": len(self._buffers[eid]),
                "mean_inter_arrival_ms": float(np.mean(iats) * 1000) if iats else 0.0,
                "std_inter_arrival_ms": float(np.std(iats) * 1000) if iats else 0.0,
                "latest_distance_m": self._distances[eid],
                "last_timestamp": self._last_timestamp[eid],
            }

        # Overall stats
        total_frames = sum(self._frame_counts.values())
        overall = {
            "total_frames": total_frames,
            "n_esps": len(self.esp_configs),
            "window_ms": self.window_ms,
            "fs": self.fs,
        }

        return {"per_esp": per_esp, "overall": overall}


# ---------------------------------------------------------------------------
# Self-test
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    import matplotlib.pyplot as plt

    logging.basicConfig(level=logging.INFO)

    print("=" * 60)
    print("Multi-ESP FFT Processing — Self-Test")
    print("=" * 60)

    rng = np.random.default_rng(123)

    # --- Test 1: Phase smoothing ---
    print("\n[Test 1] Savitzky-Golay phase smoothing (1-D)")
    t = np.linspace(0, 1, 100)
    phase_clean = 2 * np.pi * 5 * t  # 5 Hz ramp
    phase_noisy = phase_clean + rng.normal(0, 0.3, size=t.shape)
    phase_smoothed = phase_smooth_savgol(phase_noisy, window_ms=100, fs=100)
    rmse_raw = np.sqrt(np.mean((phase_noisy - phase_clean) ** 2))
    rmse_smooth = np.sqrt(np.mean((phase_smoothed - phase_clean) ** 2))
    print(f"  RMSE raw: {rmse_raw:.4f}")
    print(f"  RMSE smoothed: {rmse_smooth:.4f}")
    assert rmse_smooth < rmse_raw, "Smoothing should reduce RMSE"
    print("  PASS: Smoothing reduces noise")

    # --- Test 2: Phase smoothing (2-D) ---
    print("\n[Test 2] Savitzky-Golay phase smoothing (2-D)")
    phase_2d = np.random.randn(100, 52).cumsum(axis=0)
    phase_2d_smoothed = phase_smooth_savgol(phase_2d, window_ms=100, fs=100)
    assert phase_2d_smoothed.shape == phase_2d.shape
    print(f"  Input shape: {phase_2d.shape}")
    print(f"  Output shape: {phase_2d_smoothed.shape} OK")

    # --- Test 3: Phase slope estimation ---
    print("\n[Test 3] Phase slope estimation")
    n_sc = 52
    bw = 20e6
    delta_f = bw / n_sc
    freqs = (np.arange(n_sc) - (n_sc - 1) / 2) * delta_f
    true_tof = 50e-9  # 50 ns
    true_slope = -2 * np.pi * true_tof
    phase_ideal = true_slope * freqs
    slope_est = estimate_phase_slope(phase_ideal, freqs)
    print(f"  True slope: {true_slope:.6e} rad/Hz")
    print(f"  Estimated slope: {slope_est:.6e} rad/Hz")
    assert np.abs(slope_est - true_slope) < 1e-10, "Slope estimation failed"
    print("  PASS")

    # --- Test 4: CSI to ToF ---
    print("\n[Test 4] CSI to ToF conversion")
    csi_test = np.exp(1j * phase_ideal)
    tof_est = csi_to_tof(csi_test, bw_hz=bw)
    print(f"  True ToF: {true_tof * 1e9:.1f} ns")
    print(f"  Estimated ToF: {tof_est * 1e9:.1f} ns")
    assert np.abs(tof_est - true_tof) < 1e-10, "ToF estimation failed"
    print("  PASS")

    # --- Test 5: CSI to distance ---
    print("\n[Test 5] CSI to distance")
    true_dist = 3e8 * true_tof / 2
    dist_est = csi_to_distance(csi_test, bw_hz=bw)
    print(f"  True distance: {true_dist:.2f} m")
    print(f"  Estimated distance: {dist_est:.2f} m")
    assert np.abs(dist_est - true_dist) < 1e-2, "Distance estimation failed"
    print("  PASS")

    # --- Test 6: Distance clamping ---
    print("\n[Test 6] Distance clamping")
    phase_huge = -2 * np.pi * 1e-6 * freqs  # huge ToF
    csi_huge = np.exp(1j * phase_huge)
    dist_clamped = csi_to_distance(csi_huge, bw_hz=bw)
    print(f"  Clamped distance: {dist_clamped:.2f} m (should be <= 50)")
    assert dist_clamped <= 50.0, "Distance not clamped"
    print("  PASS")

    # --- Test 7: FFT smoothing ---
    print("\n[Test 7] FFT-based CSI smoothing")
    frames = [rng.normal(size=52) + 1j * rng.normal(size=52) for _ in range(64)]
    spec = fft_smooth_csi(frames, n_fft=256)
    print(f"  Magnitude spectrum shape: {spec.shape}")
    assert spec.shape == (52, 129), f"Expected (52, 129), got {spec.shape}"
    print("  PASS")

    # --- Test 8: Timestamp alignment ---
    print("\n[Test 8] ESP timestamp alignment")
    stream_a = [
        {"timestamp": 0.0, "csi": np.ones(52)},
        {"timestamp": 0.01, "csi": np.ones(52)},
        {"timestamp": 0.02, "csi": np.ones(52)},
        {"timestamp": 0.03, "csi": np.ones(52)},
    ]
    stream_b = [
        {"timestamp": 0.015, "csi": np.ones(52)},
        {"timestamp": 0.025, "csi": np.ones(52)},
        {"timestamp": 0.035, "csi": np.ones(52)},
    ]
    streams = {"esp_a": stream_a, "esp_b": stream_b}
    aligned = align_esp_timestamps(streams)
    print(f"  Stream A before: {len(stream_a)} after: {len(aligned['esp_a'])}")
    print(f"  Stream B before: {len(stream_b)} after: {len(aligned['esp_b'])}")
    print("  PASS")

    # --- Test 9: CFO drift compensation ---
    print("\n[Test 9] CFO drift compensation")
    t_drift = np.linspace(0, 1, 100)
    drift = 2.0 * t_drift  # linear drift of 2 rad/s
    phase_with_drift = np.sin(2 * np.pi * 2 * t_drift) + drift
    phase_comp = compensate_cfo_drift(phase_with_drift, t_drift)
    # After compensation, linear trend should be removed
    coeffs_after = np.polyfit(t_drift, phase_comp, 1)
    print(f"  Linear coefficient after compensation: {coeffs_after[0]:.6f} (should be ~0)")
    assert np.abs(coeffs_after[0]) < 0.1, "Drift not fully compensated"
    print("  PASS")

    # --- Test 10: MultiESPProcessor ---
    print("\n[Test 10] MultiESPProcessor class")
    configs = [
        ESPConfig(esp_id="esp_1", ip="192.168.1.100"),
        ESPConfig(esp_id="esp_2", ip="192.168.1.101"),
    ]
    processor = MultiESPProcessor(configs, window_ms=100, fs=100)

    # Feed frames
    n_sc = 52
    n_frames = 15  # need at least window_len frames
    results = []
    for i in range(n_frames):
        for cfg in configs:
            csi = np.exp(1j * rng.uniform(-np.pi, np.pi, size=n_sc))
            frame = {"timestamp": i * 0.01, "csi": csi}
            result = processor.process_frame(cfg.esp_id, frame)
            if result is not None:
                results.append(result)

    distances = processor.get_smoothed_distances()
    status = processor.get_sync_status()
    print(f"  Processed {n_frames} frames per ESP")
    print(f"  Smoothed outputs: {len(results)}")
    print(f"  Distances: {distances}")
    print(f"  Overall status: {status['overall']}")
    assert "esp_1" in distances and "esp_2" in distances
    print("  PASS")

    # --- Optional visualization ---
    try:
        fig, axes = plt.subplots(2, 1, figsize=(10, 6))

        axes[0].plot(t, phase_noisy, alpha=0.5, label="Noisy")
        axes[0].plot(t, phase_smoothed, linewidth=2, label="Smoothed")
        axes[0].plot(t, phase_clean, "k--", label="True")
        axes[0].set_xlabel("Time (s)")
        axes[0].set_ylabel("Phase (rad)")
        axes[0].set_title("Savitzky-Golay Phase Smoothing")
        axes[0].legend()
        axes[0].grid(True)

        axes[1].plot(t_drift, phase_with_drift, alpha=0.5, label="With drift")
        axes[1].plot(t_drift, phase_comp, linewidth=2, label="After CFO comp")
        axes[1].set_xlabel("Time (s)")
        axes[1].set_ylabel("Phase (rad)")
        axes[1].set_title("CFO Drift Compensation")
        axes[1].legend()
        axes[1].grid(True)

        plt.tight_layout()
        plt.savefig("/mnt/agents/output/csi-tracking-system/src/multi_esp_test_plot.png")
        print("\n  Plots saved to multi_esp_test_plot.png")
    except Exception as e:
        print(f"Plotting failed: {e}")
