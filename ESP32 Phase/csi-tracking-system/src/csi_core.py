"""ESP32 CSI Data Acquisition & Preprocessing Module

Handles UDP streaming, binary frame parsing, phase unwrapping,
Hampel filtering, phase sanitization, and amplitude normalization.

Example::

    source = CSISource("192.168.1.100", 5005)
    source.start()
    window = source.get_window(n_samples=100)   # (100, 52, 2)
    source.stop()
"""

from __future__ import annotations

import logging
import socket
import struct
import threading
from collections import deque
from typing import Any, Deque, Dict, List, Optional, Tuple

import numpy as np

logger = logging.getLogger(__name__)

# --------------------------------------------------------------------------- #
# Constants
# --------------------------------------------------------------------------- #

CSI_MAGIC_HEADER: int = 0xC5110001
"""4-byte little-endian magic number prepended to every ESP32 CSI frame."""

_HEADER_FMT: str = "<I d i I"
"""Struct format:
    - I : uint32  magic
    - d : float64 timestamp (seconds since epoch)
    - i : int32   RSSI
    - I : uint32  CSI payload length (number of *bytes*, not samples)
"""
_HEADER_SIZE: int = struct.calcsize(_HEADER_FMT)  # 20 bytes


# --------------------------------------------------------------------------- #
# Frame parsing
# --------------------------------------------------------------------------- #

def parse_csi_frame(data: bytes) -> Dict[str, Any]:
    """Parse an ESP32 CSI binary frame.

    Frame layout (little-endian)::

        [magic: uint32] [timestamp: float64] [rssi: int32] [csi_len: uint32] [csi_data: N bytes]

    The *csi_data* region contains pairs of signed 8-bit integers
    (I, Q) — one pair per subcarrier.  Amplitude and phase are
    computed as:

        amplitude = sqrt(I^2 + Q^2)
        phase     = atan2(Q, I)

    Args:
        data: Raw byte string received over UDP.

    Returns:
        Dictionary with keys:

        - ``timestamp`` (float):  Seconds since epoch.
        - ``rssi`` (int):         RSSI in dBm.
        - ``csi_bytes`` (bytes):  Raw CSI payload (uninterpreted).
        - ``amplitude`` (np.ndarray): 1-D float64 array of per-subcarrier amplitudes.
        - ``phase`` (np.ndarray):     1-D float64 array of per-subcarrier phases [rad].

    Raises:
        ValueError: If the frame is too short, the magic header mismatches,
                    or the CSI payload size is not an even number of bytes.
    """
    if len(data) < _HEADER_SIZE:
        raise ValueError(
            f"Frame too short: {len(data)} bytes, expected at least {_HEADER_SIZE}"
        )

    magic, timestamp, rssi, csi_len = struct.unpack_from(_HEADER_FMT, data, 0)

    if magic != CSI_MAGIC_HEADER:
        raise ValueError(
            f"Magic header mismatch: got 0x{magic:08X}, expected 0x{CSI_MAGIC_HEADER:08X}"
        )

    payload_offset = _HEADER_SIZE
    expected_total = payload_offset + csi_len
    if len(data) < expected_total:
        raise ValueError(
            f"Incomplete frame: payload claims {csi_len} bytes but only "
            f"{len(data) - payload_offset} available"
        )

    csi_bytes = data[payload_offset:payload_offset + csi_len]

    if csi_len % 2 != 0:
        raise ValueError(
            f"CSI payload length must be even (I/Q pairs), got {csi_len}"
        )

    n_subcarriers = csi_len // 2
    iq = np.frombuffer(csi_bytes, dtype=np.int8).astype(np.float64)
    i_vals = iq[0::2]
    q_vals = iq[1::2]

    amplitude = np.sqrt(i_vals * i_vals + q_vals * q_vals)
    phase = np.arctan2(q_vals, i_vals)

    return {
        "timestamp": float(timestamp),
        "rssi": int(rssi),
        "csi_bytes": csi_bytes,
        "amplitude": amplitude,
        "phase": phase,
    }


# --------------------------------------------------------------------------- #
# Signal-processing utilities
# --------------------------------------------------------------------------- #

def hampel_filter(data: np.ndarray, window: int = 5, n_sigma: float = 3.0) -> np.ndarray:
    """Hampel (MAD-based) outlier filter applied per subcarrier.

    For each sample and each subcarrier, a local window of size
    ``2*window + 1`` is examined.  Samples whose absolute deviation
    from the window median exceeds ``n_sigma * MAD`` are replaced by
    that median.

    Parameters
    ----------
    data :
        Input array.  Can be 1-D ``(n_samples,)`` or 2-D
        ``(n_samples, n_subcarriers)``.  Filtering is applied **along
        axis 0** (per subcarrier).
    window :
        Half-width of the sliding window (default 5 → window size 11).
    n_sigma :
        Number of median-absolute-deviations above which a point is
        considered an outlier.

    Returns
    -------
    np.ndarray
        Filtered array with the same shape as *data*.
    """
    if data.ndim not in (1, 2):
        raise ValueError(f"hampel_filter expects 1-D or 2-D array, got shape {data.shape}")
    if window < 1:
        raise ValueError("window must be >= 1")

    original_shape = data.shape
    if data.ndim == 1:
        data = data.reshape(-1, 1)

    n_samples, n_subcarriers = data.shape
    cleaned = data.copy()
    pad = window

    # Pad with edge values so boundaries are handled gracefully
    padded = np.pad(data, ((pad, pad), (0, 0)), mode="edge")

    for k in range(n_subcarriers):
        col = padded[:, k]
        for i in range(n_samples):
            win = col[i:i + 2 * window + 1]
            med = np.median(win)
            mad = np.median(np.abs(win - med))
            # 1.4826 converts MAD to std-dev estimate for normal distribution
            threshold = n_sigma * 1.4826 * mad
            if np.abs(data[i, k] - med) > threshold:
                cleaned[i, k] = med

    return cleaned.reshape(original_shape)


def phase_sanitize(phase: np.ndarray) -> np.ndarray:
    """Remove SFO / CFO residual by subtracting a linear fit across subcarriers.

    For each CSI sample (row), a line ``phase_k ≈ a·k + b`` is fitted
    against the subcarrier index *k*.  The fitted line is subtracted,
    leaving only the phase perturbations caused by the propagation
    channel.

    Parameters
    ----------
    phase :
        2-D array of shape ``(n_samples, n_subcarriers)``.

    Returns
    -------
    np.ndarray
        Sanitized phase with the same shape as *phase*.
    """
    if phase.ndim != 2:
        raise ValueError(f"phase_sanitize expects 2-D array, got shape {phase.shape}")

    n_samples, n_subcarriers = phase.shape
    k = np.arange(n_subcarriers)
    sanitized = np.empty_like(phase)

    for i in range(n_samples):
        coeffs = np.polyfit(k, phase[i], 1)  # linear fit: [a, b]
        fitted = coeffs[0] * k + coeffs[1]
        sanitized[i] = phase[i] - fitted

    return sanitized


def unwrap_phase(phase: np.ndarray) -> np.ndarray:
    """Resolve 2π phase discontinuities along the subcarrier dimension.

    Parameters
    ----------
    phase :
        Input phase array.  Unwrapping is performed along the **last**
        axis (``axis=-1``), which is assumed to be the subcarrier
        dimension.

    Returns
    -------
    np.ndarray
        Unwrapped phase with the same shape as *phase*.
    """
    return np.unwrap(phase, axis=-1)


def normalize_amplitude(amp: np.ndarray) -> np.ndarray:
    """Normalise amplitude by the mean across subcarriers for each sample.

    Parameters
    ----------
    amp :
        2-D array of shape ``(n_samples, n_subcarriers)``.

    Returns
    -------
    np.ndarray
        Normalised amplitude with the same shape as *amp*.
        Each row is divided by its mean (with epsilon for stability).
    """
    if amp.ndim != 2:
        raise ValueError(f"normalize_amplitude expects 2-D array, got shape {amp.shape}")
    mean_per_sample = np.mean(amp, axis=1, keepdims=True)
    return amp / (mean_per_sample + 1e-12)


# --------------------------------------------------------------------------- #
# Threaded UDP source
# --------------------------------------------------------------------------- #

class CSISource:
    """Threaded UDP listener that receives and buffers ESP32 CSI frames.

    The background thread continuously reads UDP packets, parses the
    embedded CSI frames, and stores (amplitude, phase) pairs in a
    thread-safe ring buffer.  The consumer calls :meth:`get_window` to
    retrieve the most recent *N* samples.

    Parameters
    ----------
    ip : str
        Local IP address to bind the UDP socket to (``"0.0.0.0"`` to
        listen on all interfaces).
    port : int
        UDP port number.
    buffer_maxlen : int, optional
        Maximum number of CSI entries to retain in the ring buffer.
        Older entries are automatically discarded.  Default 10_000.
    """

    def __init__(self, ip: str, port: int, buffer_maxlen: int = 10_000) -> None:
        self._ip = ip
        self._port = port
        self._buffer: Deque[Dict[str, Any]] = deque(maxlen=buffer_maxlen)
        self._lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None
        self._sock: Optional[socket.socket] = None
        self._running = threading.Event()
        self._packets_received = 0
        self._packets_failed = 0

    # ------------------------------------------------------------------ #

    def start(self) -> None:
        """Create the UDP socket and launch the background receiver thread.

        This method is idempotent — calling it while already running is
        a no-op.
        """
        if self._running.is_set():
            logger.warning("CSISource already running — ignoring start()")
            return

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self._ip, self._port))
        self._sock.settimeout(1.0)  # 1-second timeout so the loop can exit

        self._running.set()
        self._thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._thread.start()
        logger.info("CSISource started on %s:%d", self._ip, self._port)

    def _receive_loop(self) -> None:
        """Background thread body — receives packets until :meth:`stop`."""
        while self._running.is_set():
            try:
                data, addr = self._sock.recvfrom(4096)  # type: ignore[union-attr]
            except socket.timeout:
                continue
            except OSError:
                # Socket closed (expected during stop())
                break

            try:
                frame = parse_csi_frame(data)
            except ValueError as exc:
                self._packets_failed += 1
                logger.debug("Frame parse error from %s: %s", addr, exc)
                continue

            entry = {
                "timestamp": frame["timestamp"],
                "rssi": frame["rssi"],
                "amplitude": frame["amplitude"],
                "phase": frame["phase"],
            }
            with self._lock:
                self._buffer.append(entry)
            self._packets_received += 1

    def get_window(self, n_samples: int) -> np.ndarray:
        """Return the last *N* buffered samples as a stacked array.

        The returned array has shape ``(n_samples, n_subcarriers, 2)``
        where the last dimension holds ``[amplitude, phase]``.

        If fewer than *n_samples* are currently buffered, the oldest
        samples are zero-padded at the beginning.

        Parameters
        ----------
        n_samples :
            Number of recent samples to retrieve.

        Returns
        -------
        np.ndarray
            Array of shape ``(n_samples, n_subcarriers, 2)``.

        Raises
        ------
        RuntimeError
            If the source has not been started.
        """
        if not self._running.is_set():
            raise RuntimeError("CSISource is not running — call start() first")

        with self._lock:
            buf_list = list(self._buffer)

        if not buf_list:
            logger.warning("Buffer empty — cannot return window")
            return np.empty((0, 0, 2), dtype=np.float64)

        n_subcarriers = buf_list[0]["amplitude"].shape[0]

        # Take the most recent n_samples (or all available)
        available = min(n_samples, len(buf_list))
        recent = buf_list[-available:]

        window = np.zeros((n_samples, n_subcarriers, 2), dtype=np.float64)
        for i, entry in enumerate(recent):
            idx = n_samples - available + i
            window[idx, :, 0] = entry["amplitude"]
            window[idx, :, 1] = entry["phase"]

        return window

    def stop(self) -> None:
        """Signal the receiver thread to exit and close the socket."""
        if not self._running.is_set():
            return

        self._running.clear()

        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

        if self._thread is not None:
            self._thread.join(timeout=3.0)
            if self._thread.is_alive():
                logger.warning("CSISource receiver thread did not exit cleanly")
            self._thread = None

        logger.info(
            "CSISource stopped — received=%d, failed=%d, buffered=%d",
            self._packets_received,
            self._packets_failed,
            len(self._buffer),
        )

    @property
    def is_running(self) -> bool:
        """Whether the background thread is active."""
        return self._running.is_set()

    def stats(self) -> Dict[str, int]:
        """Return packet reception statistics."""
        return {
            "received": self._packets_received,
            "failed": self._packets_failed,
            "buffered": len(self._buffer),
        }


# --------------------------------------------------------------------------- #
# Quick self-test
# --------------------------------------------------------------------------- #

if __name__ == "__main__":
    # Test with synthetic CSI data
    n_subcarriers = 52
    n_samples = 100
    phase = np.random.randn(n_samples, n_subcarriers).cumsum(axis=1)
    amp = np.abs(np.random.randn(n_samples, n_subcarriers) + 1)

    # Test filters
    clean_phase = hampel_filter(phase)
    sanitized = phase_sanitize(clean_phase)
    unwrapped = unwrap_phase(sanitized)
    norm_amp = normalize_amplitude(amp)
    print(f"Phase shape: {unwrapped.shape}, Amplitude shape: {norm_amp.shape}")
