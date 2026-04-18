#!/usr/bin/env python3
"""
Unified CSI OpenGL Viewer with MAC Address Multi-Node Support
==============================================================
A complete Python viewer application that receives Wi-Fi Channel State
Information (CSI) over UART, parses MAC-addressed frames, and visualizes
multi-node CSI data with stereo camera overlay.

Protocol Support:
  - CSIv1: backward-compatible, no MAC address (CSIv1,<seq>,<rssi>,<raw_len>,<bins>,A,<amp...>,P,<phase...>)
  - CSIv2: with MAC address       (CSIv2,<seq>,<macaddr>,<rssi>,<raw_len>,<bins>,A,<amp...>,P,<phase...>)

Architecture:
  - Main Thread    : OpenGL/PyGame window (1200x600), render camera + CSI overlay, keyboard input
  - Serial Thread  : pyserial at 921600 baud, reads/parses CSI frames, pushes to CSIManager
  - Detection Proc : separate Python process for YOLO/DeepSORT (avoids GIL blocking)

Keyboard Controls:
  Q / ESC  : quit
  LEFT/RIGHT: cycle active CSI node for display
  R        : reset CSI sequence counter (sends 'R' to ESP32)
  I        : request device info (sends 'I' to ESP32)
  D        : toggle detection on/off
  S        : save current frame + CSI to file
  1-4      : jump to specific node by index

Dependencies:
  pyserial>=3.5, numpy>=1.21, opencv-python>=4.5, pygame>=2.1,
  PyOpenGL>=3.1, PyOpenGL-accelerate>=3.1, scipy>=1.7

Author: StarkHacks Team
License: MIT
"""

from __future__ import annotations

import argparse
import collections
import json
import multiprocessing
import os
import queue
import sys
import threading
import time
import traceback
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Deque, Dict, List, Optional, Tuple

import numpy as np

# ---------------------------------------------------------------------------
# Optional dependency handling
# ---------------------------------------------------------------------------
try:
    import serial
except ImportError:
    serial = None  # type: ignore

try:
    import cv2
except ImportError:
    cv2 = None  # type: ignore

try:
    import pygame
    from pygame.locals import *
except ImportError:
    pygame = None  # type: ignore

try:
    from OpenGL.GL import *
    from OpenGL.GLU import *
except ImportError:
    gl = glu = None  # type: ignore

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
APP_NAME = "Unified CSI OpenGL Viewer"
APP_VERSION = "2.0.0"
WINDOW_WIDTH = 1200
WINDOW_HEIGHT = 600
CAM_WIDTH = 640
CAM_HEIGHT = 480
COMPOSITE_WIDTH = CAM_WIDTH * 2   # 1280
COMPOSITE_HEIGHT = CAM_HEIGHT     # 480
CSI_OVERLAY_W = 400
CSI_OVERLAY_H = 150
RING_BUFFER_SIZE = 256
DEFAULT_BAUD = 921600
DEFAULT_FPS = 30
TARGET_CSI_FPS = 1000             # CSI collection as fast as serial allows

# Stereo parallax / depth constants (placeholder calibration values)
FOCAL_LENGTH_PX = 500.0           # pixels -- approximate webcam focal length
BASELINE_M = 0.12                 # metres -- typical webcam side-by-side spacing

# Colours (RGBA normalised 0-1)
COLOUR_BG = (0.05, 0.05, 0.08, 1.0)
COLOUR_OVERLAY_BG = (0.0, 0.0, 0.0, 0.65)
COLOUR_WAVE_AMP = (1.0, 0.2, 0.2, 1.0)    # red  -- amplitude
COLOUR_WAVE_FFT = (1.0, 0.9, 0.2, 1.0)    # yellow -- FFT magnitude
COLOUR_TEXT = (0.9, 0.9, 0.9, 1.0)
COLOUR_TEXT_DIM = (0.6, 0.6, 0.6, 1.0)
COLOUR_ACTIVE_NODE = (0.3, 0.8, 1.0, 1.0)
COLOUR_BBOX = (0.0, 1.0, 0.0, 1.0)        # green detection box
COLOUR_TRACK = (0.0, 0.7, 1.0, 1.0)       # cyan track label


# ============================================================================
# Data Structures
# ============================================================================

@dataclass
class CSIFrame:
    """
    Parsed CSI frame structure.

    Attributes:
        seq:        Sequence number (monotonically increasing from transmitter).
        mac:        Source MAC address string (e.g. '3c:71:bf:4a:5e:14') or None for v1.
        rssi:       Received Signal Strength Indicator in dBm.
        raw_len:    Raw CSI data length in bytes.
        bins:       Number of subcarrier bins (amplitude / phase array length).
        amplitudes: Numpy float32 array of amplitude values per subcarrier.
        phases:     Numpy float32 array of phase values per subcarrier (radians).
        timestamp:  Local reception timestamp (time.time()).
    """
    seq: int
    mac: Optional[str]
    rssi: int
    raw_len: int
    bins: int
    amplitudes: np.ndarray
    phases: np.ndarray
    timestamp: float = field(default_factory=time.time)


@dataclass
class TrackedDetection:
    """
    A single tracked detection result (person or other object).

    Attributes:
        track_id:      Unique track identifier (stable across frames).
        bbox:          Bounding box (x, y, width, height) in pixels.
        centroid:      Centroid (cx, cy) in pixels.
        label:         Class label, e.g. 'person'.
        confidence:    Detection confidence 0.0-1.0.
        missed_frames: Consecutive frames this track was not detected.
    """
    track_id: int
    bbox: Tuple[int, int, int, int]
    centroid: Tuple[int, int]
    label: str
    confidence: float
    missed_frames: int = 0


# ============================================================================
# MAC Address Registry
# ============================================================================

class MACRegistry:
    """
    Loads and manages MAC-address-to-friendly-name mappings from a JSON file.

    The registry file (default: ``mac_registry.json`` in the same directory as
    this script) contains a ``nodes`` dictionary mapping MAC addresses to
    human-readable labels such as ``NODE-LEFT``.

    When ``auto_register`` is enabled (default), any MAC address not already
    present in the registry is assigned an auto-incremented unknown name
    (``NODE-UNKNOWN-1``, ``NODE-UNKNOWN-2``, ...) and persisted back to disk.

    Thread-safe via internal ``threading.Lock``.
    """

    def __init__(self, registry_path: Optional[str] = None) -> None:
        self._lock = threading.Lock()
        self._unknown_counter = 0

        # Resolve registry file path
        if registry_path is None:
            script_dir = Path(__file__).parent.resolve()
            self.registry_path = script_dir / "mac_registry.json"
        else:
            self.registry_path = Path(registry_path).resolve()

        self.nodes: Dict[str, str] = {}
        self.auto_register: bool = True
        self.unknown_prefix: str = "NODE-UNKNOWN"

        self._load()

    def _load(self) -> None:
        """Load registry from JSON file, creating defaults if missing."""
        if self.registry_path.exists():
            try:
                with open(self.registry_path, "r", encoding="utf-8") as fh:
                    data = json.load(fh)
                self.nodes = data.get("nodes", {})
                self.auto_register = data.get("auto_register", True)
                self.unknown_prefix = data.get("unknown_prefix", "NODE-UNKNOWN")
                self._seed_unknown_counter()
                return
            except (json.JSONDecodeError, OSError) as exc:
                print(f"[MACRegistry] Warning: failed to load {self.registry_path}: {exc}")
        self._write_defaults()

    def _write_defaults(self) -> None:
        """Create registry file with built-in default mappings."""
        self.nodes = {
            "3c:71:bf:4a:5e:14": "NODE-LEFT",
            "3c:71:bf:4a:5e:15": "NODE-RIGHT",
            "3c:71:bf:4a:5e:16": "NODE-BACK",
            "3c:71:bf:4a:5e:17": "NODE-FRONT",
        }
        self.auto_register = True
        self.unknown_prefix = "NODE-UNKNOWN"
        self._save()

    def _save(self) -> None:
        """Persist current registry state to JSON file."""
        try:
            data = {
                "nodes": self.nodes,
                "auto_register": self.auto_register,
                "unknown_prefix": self.unknown_prefix,
            }
            self.registry_path.parent.mkdir(parents=True, exist_ok=True)
            with open(self.registry_path, "w", encoding="utf-8") as fh:
                json.dump(data, fh, indent=2)
        except OSError as exc:
            print(f"[MACRegistry] Warning: failed to save registry: {exc}")

    def _seed_unknown_counter(self) -> None:
        """Scan existing unknown node names to set counter past highest."""
        max_idx = 0
        prefix = self.unknown_prefix + "-"
        for name in self.nodes.values():
            if name.startswith(prefix):
                try:
                    idx = int(name[len(prefix):])
                    max_idx = max(max_idx, idx)
                except ValueError:
                    pass
        self._unknown_counter = max_idx

    def resolve(self, mac: str) -> str:
        """Return friendly name for *mac*, auto-registering if needed."""
        with self._lock:
            mac = mac.lower().strip()
            if mac in self.nodes:
                return self.nodes[mac]
            if self.auto_register:
                self._unknown_counter += 1
                name = f"{self.unknown_prefix}-{self._unknown_counter}"
                self.nodes[mac] = name
                self._save()
                print(f"[MACRegistry] Auto-registered {mac} -> {name}")
                return name
            return mac

    def lookup_name(self, mac: str) -> str:
        """Return friendly name if known, else raw MAC (no auto-register)."""
        with self._lock:
            return self.nodes.get(mac.lower().strip(), mac)

    def get_all(self) -> Dict[str, str]:
        """Return copy of the full MAC -> name mapping."""
        with self._lock:
            return dict(self.nodes)


# ============================================================================
# CSI Parser
# ============================================================================

def parse_csi_line(line: str) -> Optional[CSIFrame]:
    """
    Parse a single CSI text line from the serial port.

    Supports both **CSIv1** (no MAC field) and **CSIv2** (with MAC address).

    CSIv2 format::
        CSIv2,<seq>,<macaddr>,<rssi>,<raw_len>,<bins>,A,<amp1>,...,<ampN>,P,<phase1>,...,<phaseN>
    CSIv1 format::
        CSIv1,<seq>,<rssi>,<raw_len>,<bins>,A,<amp1>,...,<ampN>,P,<phase1>,...,<phaseN>

    Args:
        line: Raw text line from serial (may contain trailing ``\\r``).

    Returns:
        :class:`CSIFrame` on success, ``None`` if malformed.
    """
    line = line.strip()
    if not line:
        return None

    if line.startswith("CSIv2,"):
        version = "v2"
        payload = line[6:]
    elif line.startswith("CSIv1,"):
        version = "v1"
        payload = line[6:]
    else:
        return None

    parts = payload.split(",")
    if len(parts) < 6:
        return None

    idx = 0
    try:
        seq = int(parts[idx]); idx += 1
    except ValueError:
        return None

    mac: Optional[str] = None
    if version == "v2":
        mac = parts[idx].lower().strip()
        idx += 1
        if len(mac.split(":")) != 6:
            return None

    try:
        rssi = int(parts[idx]); idx += 1
        raw_len = int(parts[idx]); idx += 1
        bins = int(parts[idx]); idx += 1
    except (ValueError, IndexError):
        return None

    try:
        a_marker = parts.index("A", idx)
        p_marker = parts.index("P", a_marker + 1)
    except ValueError:
        return None

    amp_strs = parts[a_marker + 1:p_marker]
    phase_strs = parts[p_marker + 1:]

    try:
        amplitudes = np.array([float(v) for v in amp_strs if v != ""], dtype=np.float32)
        phases = np.array([float(v) for v in phase_strs if v != ""], dtype=np.float32)
    except ValueError:
        return None

    if len(amplitudes) != bins or len(phases) != bins:
        if len(amplitudes) > 0 and len(phases) > 0:
            amplitudes = np.resize(amplitudes, bins)
            phases = np.resize(phases, bins)
        else:
            return None

    return CSIFrame(
        seq=seq, mac=mac, rssi=rssi, raw_len=raw_len,
        bins=bins, amplitudes=amplitudes, phases=phases,
    )


# ============================================================================
# Per-Node CSI Ring Buffer
# ============================================================================

class NodeCSI:
    """
    Per-MAC CSI ring buffer with integrated FFT caching.

    Maintains a fixed-size deque of :class:`CSIFrame` objects and
    pre-computes the FFT magnitude spectrum whenever a new frame arrives,
    so the renderer can fetch it without blocking.

    Attributes:
        mac:          MAC address string this node represents.
        name:         Friendly human-readable name (from MACRegistry).
        buffer:       ``collections.deque`` of recent CSIFrames (maxlen=256).
        fft_magnitude: Cached amplitude spectrum from most recent frame.
        latest_seq:   Highest sequence number seen.
        frame_count:  Total frames received since creation.
        last_seen:    ``time.time()`` of most recent frame.
    """

    def __init__(self, mac: str, name: str, buffer_size: int = RING_BUFFER_SIZE) -> None:
        self.mac: str = mac
        self.name: str = name
        self.buffer: Deque[CSIFrame] = collections.deque(maxlen=buffer_size)
        self.fft_magnitude: np.ndarray = np.zeros(128, dtype=np.float32)
        self.latest_seq: int = 0
        self.frame_count: int = 0
        self.last_seen: float = time.time()

    def add_frame(self, frame: CSIFrame) -> None:
        """
        Append *frame* to ring buffer and recompute FFT magnitude.

        Args:
            frame: Parsed CSI frame to store.
        """
        self.buffer.append(frame)
        self.latest_seq = frame.seq
        self.frame_count += 1
        self.last_seen = time.time()

        amps = frame.amplitudes
        if len(amps) > 1:
            self.fft_magnitude = np.abs(np.fft.rfft(amps))
        elif len(amps) == 1:
            self.fft_magnitude = np.array([abs(amps[0])], dtype=np.float32)

    def latest_frame(self) -> Optional[CSIFrame]:
        """Return the most recent CSIFrame, or ``None`` if buffer empty."""
        return self.buffer[-1] if self.buffer else None


# ============================================================================
# Thread-Safe Multi-Node CSI Manager
# ============================================================================

class CSIManager:
    """
    Thread-safe singleton managing per-node CSI ring buffers.

    All CSI frames route here.  Internally maintains a dictionary of
    :class:`NodeCSI` objects keyed by MAC address (or ``"unknown"`` for v1).

    Thread-safe: every public method acquires ``self.lock``.

    Attributes:
        nodes:      Dict mapping MAC string -> :class:`NodeCSI`.
        lock:       ``threading.Lock`` guarding ``nodes``.
        max_nodes:  Maximum number of distinct nodes to track.
    """

    def __init__(self, mac_registry: MACRegistry, max_nodes: int = 8) -> None:
        self.nodes: Dict[str, NodeCSI] = {}
        self.lock: threading.Lock = threading.Lock()
        self.max_nodes: int = max_nodes
        self._mac_registry: MACRegistry = mac_registry
        self._seq_counter: int = 0

    def register_frame(self, frame: CSIFrame) -> None:
        """
        Route *frame* to the appropriate per-node ring buffer.

        Creates a new :class:`NodeCSI` on first sight of a MAC (subject to
        ``max_nodes`` capacity).  Drops the frame silently if at capacity.

        Args:
            frame: Parsed CSI frame to store.
        """
        mac = frame.mac if frame.mac else "unknown"

        with self.lock:
            if mac not in self.nodes:
                if len(self.nodes) >= self.max_nodes:
                    print(f"[CSIManager] At capacity ({self.max_nodes}), dropping node {mac}")
                    return
                friendly = self._mac_registry.resolve(mac)
                self.nodes[mac] = NodeCSI(mac, friendly)
                print(f"[CSIManager] New node registered: {friendly} ({mac})")

            self.nodes[mac].add_frame(frame)
            self._seq_counter += 1

    def get_node(self, mac: str) -> Optional[NodeCSI]:
        """Return :class:`NodeCSI` for *mac*, or ``None``."""
        with self.lock:
            return self.nodes.get(mac)

    def get_node_latest(self, mac: str) -> Optional[CSIFrame]:
        """Return the most recent :class:`CSIFrame` for *mac*."""
        with self.lock:
            node = self.nodes.get(mac)
            return node.latest_frame() if node else None

    def get_node_spectrum(self, mac: str) -> Optional[np.ndarray]:
        """Return the cached FFT magnitude spectrum for *mac*."""
        with self.lock:
            node = self.nodes.get(mac)
            return node.fft_magnitude.copy() if node else None

    def get_all_nodes(self) -> List[str]:
        """Return list of all known MAC addresses."""
        with self.lock:
            return list(self.nodes.keys())

    def get_active_nodes(self, timeout: float = 2.0) -> List[str]:
        """
        Return MACs of nodes that have sent data within *timeout* seconds.

        Args:
            timeout: Inactivity threshold in seconds.
        """
        now = time.time()
        with self.lock:
            return [mac for mac, node in self.nodes.items()
                    if now - node.last_seen < timeout]

    def drop_stale(self, timeout: float = 10.0) -> int:
        """
        Remove nodes idle longer than *timeout* seconds.

        Returns:
            Number of nodes removed.
        """
        now = time.time()
        with self.lock:
            stale = [m for m, n in self.nodes.items() if now - n.last_seen > timeout]
            for m in stale:
                friendly = self.nodes[m].name
                del self.nodes[m]
                print(f"[CSIManager] Dropped stale node: {friendly}")
            return len(stale)

    def get_stats(self) -> Dict[str, Any]:
        """Return summary statistics for all nodes."""
        with self.lock:
            return {
                mac: {
                    "name": node.name,
                    "frames": node.frame_count,
                    "latest_seq": node.latest_seq,
                    "last_seen": node.last_seen,
                    "buffer_len": len(node.buffer),
                }
                for mac, node in self.nodes.items()
            }


# ============================================================================
# Serial Reading Thread
# ============================================================================

class SerialThread(threading.Thread):
    """
    Dedicated serial reading thread.

    Continuously reads lines from the ESP32 UART, parses CSI frames,
    and forwards them to the :class:`CSIManager`.  Also captures
    servo command echo / info lines and posts them to a ``Queue``.

    Attributes:
        port:        Serial port path (e.g. ``/dev/cu.usbserial-1130``).
        baud:        Baud rate (default 921600).
        csi_manager: Shared :class:`CSIManager` instance.
        servo_status: ``queue.Queue`` for non-CSI lines (SERVO/INFO).
        csi_fps:     Most recent measured CSI reception rate (Hz).
    """

    def __init__(
        self,
        port: str,
        baud: int,
        csi_manager: CSIManager,
        servo_status_queue: "queue.Queue[str]",
    ) -> None:
        super().__init__(daemon=True, name="SerialThread")
        self.port: str = port
        self.baud: int = baud
        self.csi_manager: CSIManager = csi_manager
        self.servo_status: "queue.Queue[str]" = servo_status_queue
        self._running: bool = True
        self._ser: Any = None
        self._fps_counter: int = 0
        self._fps_time: float = time.time()
        self.csi_fps: float = 0.0

    def run(self) -> None:
        """Main thread loop: connect, read, parse, dispatch."""
        if serial is None:
            print("[Serial] ERROR: pyserial not installed. CSI input disabled.")
            return

        while self._running:
            try:
                self._ser = serial.Serial(self.port, self.baud, timeout=0.1)
                print(f"[Serial] Opened {self.port} @ {self.baud} baud")
                break
            except serial.SerialException as exc:
                print(f"[Serial] Cannot open {self.port}: {exc}. Retrying in 2s...")
                time.sleep(2.0)

        buffer = ""
        while self._running:
            try:
                if self._ser is None or not self._ser.is_open:
                    time.sleep(0.5)
                    continue

                data = self._ser.read(self._ser.in_waiting or 1)
                if data:
                    buffer += data.decode("utf-8", errors="ignore")

                    while "\n" in buffer:
                        line, buffer = buffer.split("\n", 1)
                        line = line.strip()
                        self._handle_line(line)

                now = time.time()
                if now - self._fps_time >= 1.0:
                    self.csi_fps = self._fps_counter / (now - self._fps_time)
                    self._fps_counter = 0
                    self._fps_time = now

            except serial.SerialException as exc:
                print(f"[Serial] Error: {exc}. Reconnecting...")
                self._try_reconnect()
            except Exception as exc:
                print(f"[Serial] Unexpected error: {exc}")
                traceback.print_exc()
                time.sleep(0.5)

    def _handle_line(self, line: str) -> None:
        """Dispatch a single text line to CSI parser or servo queue."""
        if not line:
            return
        if line.startswith("CSIv1") or line.startswith("CSIv2"):
            frame = parse_csi_line(line)
            if frame is not None:
                self.csi_manager.register_frame(frame)
                self._fps_counter += 1
        elif line.startswith("SERVO") or line.startswith("INFO"):
            self.servo_status.put(line)

    def _try_reconnect(self) -> None:
        """Attempt to re-establish serial connection."""
        try:
            if self._ser and self._ser.is_open:
                self._ser.close()
        except Exception:
            pass
        time.sleep(1.0)
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=0.1)
            print(f"[Serial] Reconnected to {self.port}")
        except serial.SerialException:
            pass

    def send_command(self, cmd: str) -> None:
        """Send a command string to the ESP32 (appends ``\\n``)."""
        if self._ser and self._ser.is_open:
            try:
                self._ser.write(f"{cmd}\n".encode())
            except serial.SerialException as exc:
                print(f"[Serial] Write error: {exc}")

    def stop(self) -> None:
        """Signal the thread to stop and close the serial port."""
        self._running = False
        if self._ser and self._ser.is_open:
            try:
                self._ser.close()
            except Exception:
                pass


# ============================================================================
# Simple IOU-Based Tracker
# ============================================================================

class IOUTracker:
    """
    Lightweight multi-object tracker using Intersection-over-Union matching.

    This is a simplified tracker that does **not** require DeepSORT;
    it associates detections across frames by greedy IOU matching.
    Suitable for ``hog`` backend or as a lightweight fallback.

    Attributes:
        tracks:        Dict mapping track_id -> :class:`TrackedDetection`.
        _next_id:      Monotonically increasing track counter.
        _max_missed:   Frames before a track is deleted.
        _iou_threshold: Minimum IOU to associate detection with existing track.
    """

    def __init__(self, max_missed: int = 30, iou_threshold: float = 0.3) -> None:
        self.tracks: Dict[int, TrackedDetection] = {}
        self._next_id: int = 1
        self._max_missed: int = max_missed
        self._iou_threshold: float = iou_threshold

    def update(self, detections: List[Dict[str, Any]]) -> List[TrackedDetection]:
        """
        Update tracks with new detections and return active tracks.

        Args:
            detections: List of detection dicts with keys ``bbox`` (x,y,w,h),
                        ``confidence``, ``label``.

        Returns:
            List of active (non-expired) :class:`TrackedDetection` objects.
        """
        new_dets = []
        for d in detections:
            x, y, w, h = d["bbox"]
            cx, cy = int(x + w // 2), int(y + h // 2)
            new_dets.append({
                "bbox": (int(x), int(y), int(w), int(h)),
                "centroid": (cx, cy),
                "confidence": float(d.get("confidence", 1.0)),
                "label": d.get("label", "person"),
            })

        matched_track_ids = set()
        matched_det_indices = set()

        track_items = sorted(
            self.tracks.items(),
            key=lambda kv: kv[1].missed_frames,
        )

        for tid, track in track_items:
            best_iou = self._iou_threshold
            best_idx = -1
            for idx, det in enumerate(new_dets):
                if idx in matched_det_indices:
                    continue
                iou = self._iou(track.bbox, det["bbox"])
                if iou > best_iou:
                    best_iou = iou
                    best_idx = idx
            if best_idx >= 0:
                det = new_dets[best_idx]
                track.bbox = det["bbox"]
                track.centroid = det["centroid"]
                track.confidence = det["confidence"]
                track.label = det["label"]
                track.missed_frames = 0
                matched_track_ids.add(tid)
                matched_det_indices.add(best_idx)

        for idx, det in enumerate(new_dets):
            if idx not in matched_det_indices:
                tid = self._next_id
                self._next_id += 1
                self.tracks[tid] = TrackedDetection(
                    track_id=tid,
                    bbox=det["bbox"],
                    centroid=det["centroid"],
                    label=det["label"],
                    confidence=det["confidence"],
                    missed_frames=0,
                )

        for tid, track in self.tracks.items():
            if tid not in matched_track_ids:
                track.missed_frames += 1

        stale = [tid for tid, t in self.tracks.items() if t.missed_frames > self._max_missed]
        for tid in stale:
            del self.tracks[tid]

        return list(self.tracks.values())

    @staticmethod
    def _iou(
        a: Tuple[int, int, int, int], b: Tuple[int, int, int, int]
    ) -> float:
        """Compute Intersection-over-Union of two bounding boxes."""
        ax, ay, aw, ah = a
        bx, by, bw, bh = b

        a_x2, a_y2 = ax + aw, ay + ah
        b_x2, b_y2 = bx + bw, by + bh

        ix = max(0, min(a_x2, b_x2) - max(ax, bx))
        iy = max(0, min(a_y2, b_y2) - max(ay, by))
        inter = ix * iy

        area_a = aw * ah
        area_b = bw * bh
        union = area_a + area_b - inter
        return inter / union if union > 0 else 0.0


# ============================================================================
# Detection Process (multiprocessing)
# ============================================================================

class DetectionProcess(multiprocessing.Process):
    """
    Separate process for object detection to avoid GIL contention.

    Reads frames from *frame_queue*, runs detection every *interval* frames,
    and posts results to *result_queue*.

    For ``yolo_deepsort`` backend, the heavy YOLO/DeepSORT imports happen
    inside this process (which should run under its own venv if needed).
    For ``hog`` backend, OpenCV's HOGDescriptor is used directly.

    Attributes:
        frame_queue: ``multiprocessing.Queue`` of incoming frames (numpy arrays).
        result_queue: ``multiprocessing.Queue`` for detection results.
        backend:     ``'yolo_deepsort'`` | ``'hog'`` | ``'none'``.
        interval:    Run detection every N frames (skip intermediate).
    """

    def __init__(
        self,
        frame_queue: "multiprocessing.Queue",
        result_queue: "multiprocessing.Queue",
        backend: str = "hog",
        interval: int = 1,
    ) -> None:
        super().__init__(daemon=True, name="DetectionProcess")
        self.frame_queue = frame_queue
        self.result_queue = result_queue
        self.backend = backend
        self.interval = interval
        self._stop_event = multiprocessing.Event()

    def run(self) -> None:
        """Process main loop: import backend, consume frames, post results."""
        print(f"[Detection] Process started (backend={self.backend})")

        detector = None
        if self.backend == "yolo_deepsort":
            detector = self._init_yolo()
        elif self.backend == "hog":
            detector = self._init_hog()
        else:
            print("[Detection] Backend is 'none' -- detector idle.")

        frame_counter = 0
        while not self._stop_event.is_set():
            try:
                frame = self.frame_queue.get(timeout=0.5)
                frame_counter += 1

                if self.backend == "none" or detector is None:
                    continue
                if frame_counter % self.interval != 0:
                    continue

                if self.backend == "hog":
                    result = self._hog_detect(detector, frame)
                elif self.backend == "yolo_deepsort":
                    result = self._yolo_detect(detector, frame)
                else:
                    continue

                self.result_queue.put(result, timeout=0.5)
            except queue.Empty:
                continue
            except Exception as exc:
                print(f"[Detection] Error: {exc}")
                traceback.print_exc()

        print("[Detection] Process exiting.")

    def stop(self) -> None:
        """Signal the process to terminate gracefully."""
        self._stop_event.set()

    @staticmethod
    def _init_hog() -> Any:
        """Initialise OpenCV HOG+SVM people detector."""
        hog = cv2.HOGDescriptor()
        hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        return hog

    @staticmethod
    def _hog_detect(hog: Any, frame: np.ndarray) -> Dict[str, Any]:
        """Run HOG detection on *frame*."""
        scale = 1.05
        found, weights = hog.detectMultiScale(
            frame,
            winStride=(8, 8),
            padding=(4, 4),
            scale=scale,
        )
        detections = []
        for i, (x, y, w, h) in enumerate(found):
            conf = float(weights[i]) if i < len(weights) else 1.0
            detections.append({
                "bbox": (int(x), int(y), int(w), int(h)),
                "confidence": conf,
                "label": "person",
            })
        return {"detections": detections, "timestamp": time.time()}

    @staticmethod
    def _init_yolo() -> Any:
        """
        Initialise YOLOv7 + DeepSORT tracker.
        """
        yolo_path = os.path.join(
            os.path.dirname(__file__), "..", "third_party", "YOLOv7-DeepSORT-Human-Tracking"
        )
        yolo_path = os.path.abspath(yolo_path)
        if yolo_path not in sys.path:
            sys.path.insert(0, yolo_path)

        try:
            from tracker import DeepSortTracker  # type: ignore
            print("[Detection] Loaded DeepSortTracker from YOLOv7 submodule")
            return DeepSortTracker()
        except ImportError:
            print("[Detection] WARNING: DeepSortTracker not found")
            return None

    @staticmethod
    def _yolo_detect(tracker: Any, frame: np.ndarray) -> Dict[str, Any]:
        """Run YOLO+DeepSORT detection on *frame*."""
        if tracker is None:
            return {"detections": [], "timestamp": time.time()}
        try:
            dets = tracker.update(frame)
            detections = []
            for d in dets:
                detections.append({
                    "bbox": d.get("bbox", d.get("loc", (0, 0, 0, 0))),
                    "confidence": d.get("confidence", 1.0),
                    "label": d.get("label", "person"),
                    "track_id": d.get("track_id", -1),
                })
            return {"detections": detections, "timestamp": time.time()}
        except Exception as exc:
            print(f"[Detection] YOLO error: {exc}")
            return {"detections": [], "timestamp": time.time()}


# ============================================================================
# OpenGL Renderer
# ============================================================================

class OpenGLRenderer:
    """
    OpenGL-based renderer for camera composite + CSI overlay panel.

    Uses ``pygame`` for window management and ``PyOpenGL`` for all drawing.
    The camera frames are uploaded as OpenGL textures each frame; the CSI
    overlay (spectrum waveforms, text, bounding boxes) is drawn as textured
    quads and line strips on top.

    Attributes:
        width:  Window width in pixels.
        height: Window height in pixels.
        overlay_x: Left position of the CSI overlay panel.
        overlay_y: Top position of the CSI overlay panel.
    """

    def __init__(
        self,
        width: int = WINDOW_WIDTH,
        height: int = WINDOW_HEIGHT,
        overlay_x: int = 10,
        overlay_y: int = 10,
    ) -> None:
        if pygame is None:
            raise RuntimeError("pygame is required but not installed")

        self.width = width
        self.height = height
        self.overlay_x = overlay_x
        self.overlay_y = overlay_y

        # pygame / OpenGL init
        pygame.init()
        pygame.display.set_mode(
            (width, height),
            pygame.DOUBLEBUF | pygame.OPENGL | pygame.RESIZABLE,
        )
        pygame.display.set_caption(f"{APP_NAME} v{APP_VERSION}")

        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glClearColor(*COLOUR_BG)
        glViewport(0, 0, width, height)

        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        glOrtho(0.0, width, height, 0.0, -1.0, 1.0)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

        # Font setup
        font_size = 14
        try:
            self._font = pygame.font.Font(None, font_size)
            self._font_large = pygame.font.Font(None, 20)
        except Exception:
            self._font = pygame.font.SysFont("monospace", font_size)
            self._font_large = pygame.font.SysFont("monospace", 20)

        # Camera texture placeholder
        self._cam_tex: int = 0
        self._init_camera_texture()

        # Amplitude auto-scaling state
        self._amp_scale: float = 1.0
        self._amp_scale_target: float = 1.0
        self._last_amp_update: float = time.time()

    def _init_camera_texture(self) -> None:
        """Generate an OpenGL texture ID for camera frame display."""
        self._cam_tex = glGenTextures(1)
        glBindTexture(GL_TEXTURE_2D, self._cam_tex)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
        glBindTexture(GL_TEXTURE_2D, 0)

    def upload_camera_frame(self, composite: np.ndarray) -> None:
        """
        Upload a camera composite image as an OpenGL texture.

        Args:
            composite: BGR or RGB numpy array (H, W, 3) uint8.
        """
        if composite is None or composite.size == 0:
            return
        h, w = composite.shape[:2]
        if composite.shape[2] == 3:
            rgb = cv2.cvtColor(composite, cv2.COLOR_BGR2RGB)
        else:
            rgb = composite
        rgb = np.flipud(rgb)
        rgb = np.ascontiguousarray(rgb)

        glBindTexture(GL_TEXTURE_2D, self._cam_tex)
        glTexImage2D(
            GL_TEXTURE_2D, 0, GL_RGB,
            w, h, 0,
            GL_RGB, GL_UNSIGNED_BYTE, rgb,
        )
        glBindTexture(GL_TEXTURE_2D, 0)

    @staticmethod
    def draw_textured_quad(x: float, y: float, w: float, h: float, tex_id: int) -> None:
        """Draw a textured quad with the current bound texture."""
        glBindTexture(GL_TEXTURE_2D, tex_id)
        glEnable(GL_TEXTURE_2D)
        glBegin(GL_QUADS)
        glTexCoord2f(0.0, 1.0); glVertex2f(x, y)
        glTexCoord2f(1.0, 1.0); glVertex2f(x + w, y)
        glTexCoord2f(1.0, 0.0); glVertex2f(x + w, y + h)
        glTexCoord2f(0.0, 0.0); glVertex2f(x, y + h)
        glEnd()
        glDisable(GL_TEXTURE_2D)
        glBindTexture(GL_TEXTURE_2D, 0)

    @staticmethod
    def draw_rect(x: float, y: float, w: float, h: float, colour: Tuple[float, ...]) -> None:
        """Draw a filled rectangle with solid *colour* (RGBA)."""
        glColor4f(*colour)
        glBegin(GL_QUADS)
        glVertex2f(x, y)
        glVertex2f(x + w, y)
        glVertex2f(x + w, y + h)
        glVertex2f(x, y + h)
        glEnd()

    @staticmethod
    def draw_line_strip(points: List[Tuple[float, float]], colour: Tuple[float, ...], width: float = 1.0) -> None:
        """Draw a connected line strip."""
        glColor4f(*colour)
        glLineWidth(width)
        glBegin(GL_LINE_STRIP)
        for px, py in points:
            glVertex2f(px, py)
        glEnd()
        glLineWidth(1.0)

    @staticmethod
    def draw_lines(lines: List[Tuple[Tuple[float, float], Tuple[float, float]]], colour: Tuple[float, ...], width: float = 1.0) -> None:
        """Draw disconnected line segments (for bounding boxes)."""
        glColor4f(*colour)
        glLineWidth(width)
        glBegin(GL_LINES)
        for (x1, y1), (x2, y2) in lines:
            glVertex2f(x1, y1)
            glVertex2f(x2, y2)
        glEnd()
        glLineWidth(1.0)

    def draw_text(self, text: str, x: float, y: float, colour: Tuple[float, ...] = COLOUR_TEXT, large: bool = False) -> None:
        """
        Render *text* using pygame font, upload as texture, draw at (x, y).
        Coordinates are in window pixels, (0,0) at top-left.
        """
        font = self._font_large if large else self._font
        surface = font.render(text, True, (
            int(colour[0] * 255),
            int(colour[1] * 255),
            int(colour[2] * 255),
        ))
        tw, th = surface.get_size()

        tex_data = pygame.image.tostring(surface, "RGBA", True)
        tex = glGenTextures(1)
        glBindTexture(GL_TEXTURE_2D, tex)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, tw, th, 0, GL_RGBA, GL_UNSIGNED_BYTE, tex_data)

        self.draw_textured_quad(x, y, tw, th, tex)
        glDeleteTextures(1, [tex])

    def draw_spectrum(
        self,
        x: int, y: int, width: int, height: int,
        amplitudes: Optional[np.ndarray],
        fft_magnitude: Optional[np.ndarray],
    ) -> None:
        """
        Draw the CSI spectrum overlay panel at (x, y).

        Draws semi-transparent black background, red amplitude waveform,
        and yellow FFT magnitude line.
        """
        self.draw_rect(x, y, width, height, COLOUR_OVERLAY_BG)

        if amplitudes is None or len(amplitudes) == 0:
            self.draw_text("No CSI data", x + 10, y + height // 2, COLOUR_TEXT_DIM)
            return

        n = len(amplitudes)
        padding_x = 5
        plot_w = width - padding_x * 2
        plot_h = height - 10
        plot_x = x + padding_x
        plot_y = y + 5

        # Auto-scaling with 0.1s decay
        amp_max = float(np.max(np.abs(amplitudes))) if len(amplitudes) > 0 else 1.0
        if amp_max < 1e-6:
            amp_max = 1.0

        now = time.time()
        dt = now - self._last_amp_update
        self._last_amp_update = now

        decay = 0.5 ** (dt / 0.1)
        self._amp_scale_target = max(amp_max, self._amp_scale_target * decay)
        self._amp_scale = self._amp_scale_target
        if self._amp_scale < 1e-6:
            self._amp_scale = 1.0

        # Amplitude waveform (red)
        amp_points = []
        for i in range(n):
            px = plot_x + (i / max(n - 1, 1)) * plot_w
            norm = float(amplitudes[i]) / self._amp_scale
            py = plot_y + plot_h - max(0, min(norm, 1.0)) * plot_h
            amp_points.append((px, py))
        self.draw_line_strip(amp_points, COLOUR_WAVE_AMP, width=2.0)

        # FFT magnitude (yellow)
        if fft_magnitude is not None and len(fft_magnitude) > 1:
            fft_points = []
            fft_n = len(fft_magnitude)
            fft_max = float(np.max(fft_magnitude)) if len(fft_magnitude) > 0 else 1.0
            if fft_max < 1e-6:
                fft_max = 1.0
            for i in range(fft_n):
                px = plot_x + (i / max(fft_n - 1, 1)) * plot_w
                norm = float(fft_magnitude[i]) / fft_max
                py = plot_y + plot_h - max(0, min(norm, 1.0)) * plot_h
                fft_points.append((px, py))
            self.draw_line_strip(fft_points, COLOUR_WAVE_FFT, width=2.0)

        self.draw_text(f"Bins: {n}", x + width - 60, y + height - 14, COLOUR_TEXT_DIM)

    def draw_detections(
        self,
        tracks: List[TrackedDetection],
        cam_x: int = 0, cam_y: int = 0,
        cam_w: int = COMPOSITE_WIDTH, cam_h: int = COMPOSITE_HEIGHT,
    ) -> None:
        """Draw detection bounding boxes and track labels over the camera area."""
        for track in tracks:
            x, y, w, h = track.bbox
            scale_x = cam_w / COMPOSITE_WIDTH
            scale_y = cam_h / COMPOSITE_HEIGHT
            dx = cam_x + int(x * scale_x)
            dy = cam_y + int(y * scale_y)
            dw = int(w * scale_x)
            dh = int(h * scale_y)

            lines = [
                ((dx, dy), (dx + dw, dy)),
                ((dx + dw, dy), (dx + dw, dy + dh)),
                ((dx + dw, dy + dh), (dx, dy + dh)),
                ((dx, dy + dh), (dx, dy)),
            ]
            self.draw_lines(lines, COLOUR_BBOX, width=2.0)

            label_text = f"{track.label} #{track.track_id} ({track.confidence:.2f})"
            self.draw_text(label_text, dx, max(dy - 16, cam_y), COLOUR_TRACK)

            cx, cy = track.centroid
            cx = cam_x + int(cx * scale_x)
            cy = cam_y + int(cy * scale_y)
            self.draw_rect(cx - 2, cy - 2, 4, 4, COLOUR_TRACK)

    def draw_node_status(
        self, x: int, y: int,
        node_name: str, mac: str,
        seq: int, rssi: int,
        active_count: int, csi_fps: float,
        parallax: Optional[float] = None,
        depth: Optional[float] = None,
    ) -> None:
        """Draw multi-line status text block at the given position."""
        line_h = 16
        lines = [
            f"NODE: {node_name}  ({mac[:17] if mac else 'N/A'})",
            f"SEQ: {seq}    RSSI: {rssi}dBm",
            f"NODES ACTIVE: {active_count}",
            f"CSI FPS: {csi_fps:.1f}",
        ]
        if parallax is not None and depth is not None:
            lines.append(f"d: {parallax:.1f}px    z: {depth:.2f}m")

        for i, text in enumerate(lines):
            self.draw_text(text, x, y + i * line_h, COLOUR_TEXT)

    def draw_top_bar(self, x: int, y: int, node_names: List[str], active_index: int) -> None:
        """Draw the node selector top bar with left/right arrows."""
        bar_h = 24
        bar_w = CSI_OVERLAY_W
        self.draw_rect(x, y, bar_w, bar_h, (0.0, 0.0, 0.0, 0.75))

        if node_names:
            name = node_names[active_index % len(node_names)]
            arrow_left = "< " if len(node_names) > 1 else ""
            arrow_right = " >" if len(node_names) > 1 else ""
            text = f"{arrow_left}[{active_index + 1}/{len(node_names)}] {name}{arrow_right}"
            self.draw_text(text, x + 10, y + 4, COLOUR_ACTIVE_NODE)
        else:
            self.draw_text("No CSI nodes", x + 10, y + 4, COLOUR_TEXT_DIM)

    def render_frame(
        self,
        camera_composite: Optional[np.ndarray],
        active_node: Optional[NodeCSI],
        active_mac_index: int,
        all_node_names: List[str],
        active_count: int,
        csi_fps: float,
        tracks: List[TrackedDetection],
        parallax: Optional[float] = None,
        depth: Optional[float] = None,
    ) -> None:
        """
        Render one complete frame: camera + CSI overlay + detections.
        Main entry point called once per display frame.
        """
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()

        cam_display_w = COMPOSITE_WIDTH
        cam_display_h = COMPOSITE_HEIGHT
        cam_x = 0
        cam_y = 0

        if camera_composite is not None:
            self.upload_camera_frame(camera_composite)
            self.draw_textured_quad(cam_x, cam_y, cam_display_w, cam_display_h, self._cam_tex)
        else:
            self.draw_rect(0, 0, self.width, self.height, COLOUR_BG)

        if tracks:
            self.draw_detections(tracks, cam_x, cam_y, cam_display_w, cam_display_h)

        ox = self.overlay_x
        oy = self.overlay_y

        self.draw_top_bar(ox, oy, all_node_names, active_mac_index)

        spectrum_y = oy + 28
        amps = None
        fft_mag = None
        seq = 0
        rssi = 0
        node_name = "N/A"
        mac_str = ""

        if active_node is not None:
            latest = active_node.latest_frame()
            if latest is not None:
                amps = latest.amplitudes
                seq = latest.seq
                rssi = latest.rssi
                mac_str = active_node.mac
            fft_mag = active_node.fft_magnitude
            node_name = active_node.name

        self.draw_spectrum(ox, spectrum_y, CSI_OVERLAY_W, CSI_OVERLAY_H, amps, fft_mag)

        status_y = spectrum_y + CSI_OVERLAY_H + 8
        self.draw_node_status(
            ox, status_y, node_name, mac_str, seq, rssi,
            active_count, csi_fps, parallax, depth,
        )

        help_lines = [
            "Q/ESC:Quit  LEFT/RIGHT:Node  R:Reset  I:Info  D:Detect  S:Save  1-4:Jump",
        ]
        for i, hl in enumerate(help_lines):
            self.draw_text(hl, self.width - 600, self.height - 20 + i * 14, COLOUR_TEXT_DIM)

        pygame.display.flip()

    def cleanup(self) -> None:
        """Release OpenGL resources and quit pygame."""
        if self._cam_tex:
            glDeleteTextures(1, [self._cam_tex])
        pygame.quit()


# ============================================================================
# Unified Viewer -- Main Application
# ============================================================================

class UnifiedViewer:
    """
    Main application class tying together all subsystems.

    Manages:
    - Camera capture (OpenCV, two cameras side-by-side)
    - Serial thread (CSI reception)
    - Detection process (optional YOLO/HOG)
    - CSI manager (per-node ring buffers)
    - OpenGL renderer (display compositing)
    - Keyboard input handling
    - Stereo parallax / coarse depth estimation

    Usage::
        viewer = UnifiedViewer(args)
        viewer.run()
    """

    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args

        # MAC Registry & CSI Manager
        self.mac_registry = MACRegistry(args.mac_registry)
        self.csi_manager = CSIManager(self.mac_registry, max_nodes=8)

        # Serial thread
        self.servo_queue: "queue.Queue[str]" = queue.Queue()
        self.serial_thread = SerialThread(
            port=args.port,
            baud=args.baud,
            csi_manager=self.csi_manager,
            servo_status_queue=self.servo_queue,
        )

        # Detection subsystem
        self.detection_enabled: bool = args.stereo_detect
        self.detector_backend: str = args.detector_backend
        self.detect_interval: int = args.stereo_detect_interval
        self.max_missed: int = args.stereo_max_missed

        self.detection_proc: Optional[DetectionProcess] = None
        self.det_frame_queue: Optional["multiprocessing.Queue"] = None
        self.det_result_queue: Optional["multiprocessing.Queue"] = None

        # IOU tracker (used for HOG / fallback)
        self.iou_tracker = IOUTracker(max_missed=self.max_missed)
        self.tracks: List[TrackedDetection] = []

        # Camera state
        self.cam_left_idx: int = args.stereo_left
        self.cam_right_idx: int = args.stereo_right
        self.cam_left: Any = None
        self.cam_right: Any = None
        self.camera_composite: Optional[np.ndarray] = None

        # OpenGL renderer
        self.renderer = OpenGLRenderer(
            width=WINDOW_WIDTH,
            height=WINDOW_HEIGHT,
            overlay_x=args.csi_overlay_x,
            overlay_y=args.csi_overlay_y,
        )

        # Active node selection
        self.active_mac_index: int = 0
        self.all_macs: List[str] = []

        # Parallax / depth state
        self.parallax_px: Optional[float] = None
        self.depth_m: Optional[float] = None

        # State flags
        self._running: bool = True
        self._frame_counter: int = 0
        self._last_drop_check: float = time.time()
        self._saved_frame_counter: int = 0

    def _init_cameras(self) -> bool:
        """Open left and right camera devices via OpenCV."""
        if cv2 is None:
            print("[Camera] WARNING: opencv-python not installed")
            return False

        print(f"[Camera] Opening left={self.cam_left_idx}, right={self.cam_right_idx}")
        self.cam_left = cv2.VideoCapture(self.cam_left_idx)
        if self.cam_left.isOpened():
            self.cam_left.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_WIDTH)
            self.cam_left.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)

        self.cam_right = cv2.VideoCapture(self.cam_right_idx)
        if self.cam_right.isOpened():
            self.cam_right.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_WIDTH)
            self.cam_right.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)

        left_ok = self.cam_left is not None and self.cam_left.isOpened()
        right_ok = self.cam_right is not None and self.cam_right.isOpened()

        if left_ok:
            print(f"[Camera] Left camera opened ({self.cam_left_idx})")
        if right_ok:
            print(f"[Camera] Right camera opened ({self.cam_right_idx})")

        return left_ok or right_ok

    def _read_cameras(self) -> Optional[np.ndarray]:
        """
        Read frames from both cameras and composite side-by-side.
        Layout: [right_camera][left_camera].
        """
        left_frame = None
        right_frame = None

        if self.cam_left is not None and self.cam_left.isOpened():
            ok, frame = self.cam_left.read()
            if ok:
                left_frame = cv2.resize(frame, (CAM_WIDTH, CAM_HEIGHT))

        if self.cam_right is not None and self.cam_right.isOpened():
            ok, frame = self.cam_right.read()
            if ok:
                right_frame = cv2.resize(frame, (CAM_WIDTH, CAM_HEIGHT))

        if left_frame is None and right_frame is None:
            return None

        if left_frame is None:
            left_frame = np.zeros((CAM_HEIGHT, CAM_WIDTH, 3), dtype=np.uint8)
        if right_frame is None:
            right_frame = np.zeros((CAM_HEIGHT, CAM_WIDTH, 3), dtype=np.uint8)

        composite = np.hstack([right_frame, left_frame])
        return composite

    def _release_cameras(self) -> None:
        """Release camera resources."""
        if self.cam_left is not None:
            self.cam_left.release()
        if self.cam_right is not None:
            self.cam_right.release()

    def _start_detection(self) -> None:
        """Spawn the detection child process if a backend is configured."""
        if self.detector_backend == "none":
            return
        self.det_frame_queue = multiprocessing.Queue(maxsize=2)
        self.det_result_queue = multiprocessing.Queue(maxsize=2)
        self.detection_proc = DetectionProcess(
            frame_queue=self.det_frame_queue,
            result_queue=self.det_result_queue,
            backend=self.detector_backend,
            interval=self.detect_interval,
        )
        self.detection_proc.start()
        print(f"[Detection] Started '{self.detector_backend}' backend (PID {self.detection_proc.pid})")

    def _stop_detection(self) -> None:
        """Terminate the detection child process."""
        if self.detection_proc is not None:
            self.detection_proc.stop()
            self.detection_proc.join(timeout=2.0)
            if self.detection_proc.is_alive():
                self.detection_proc.terminate()
            self.detection_proc = None
        print("[Detection] Stopped.")

    def _poll_detection_results(self) -> None:
        """Check for new detection results from the child process."""
        if self.det_result_queue is None:
            return
        try:
            while not self.det_result_queue.empty():
                result = self.det_result_queue.get_nowait()
                detections = result.get("detections", [])
                self.tracks = self.iou_tracker.update(detections)
        except queue.Empty:
            pass
        except Exception as exc:
            print(f"[Detection] Result poll error: {exc}")

    def _compute_stereo_depth(self) -> None:
        """
        Compute coarse depth from stereo disparity when both cameras see
        the same tracked person.
        """
        if not self.tracks:
            self.parallax_px = None
            self.depth_m = None
            return

        right_cam_tracks = []
        left_cam_tracks = []

        for track in self.tracks:
            cx = track.centroid[0]
            if cx < CAM_WIDTH:
                right_cam_tracks.append(track)
            else:
                left_cam_tracks.append(track)

        best_parallax = None
        for rt in right_cam_tracks:
            for lt in left_cam_tracks:
                disparity = abs(lt.centroid[0] - CAM_WIDTH - rt.centroid[0])
                if disparity > 0:
                    best_parallax = disparity
                    break
            if best_parallax is not None:
                break

        if best_parallax is not None and best_parallax > 0:
            self.parallax_px = float(best_parallax)
            self.depth_m = (FOCAL_LENGTH_PX * BASELINE_M) / best_parallax
        else:
            self.parallax_px = None
            self.depth_m = None

    def _handle_keydown(self, event: Any) -> None:
        """Process pygame KEYDOWN events."""
        key = event.key

        if key == pygame.K_q or key == pygame.K_ESCAPE:
            print("[Input] Quit requested")
            self._running = False

        elif key == pygame.K_LEFT:
            if self.all_macs:
                self.active_mac_index = (self.active_mac_index - 1) % len(self.all_macs)
                node = self.csi_manager.get_node(self.all_macs[self.active_mac_index])
                name = node.name if node else "?"
                print(f"[Input] Active node: {name}")

        elif key == pygame.K_RIGHT:
            if self.all_macs:
                self.active_mac_index = (self.active_mac_index + 1) % len(self.all_macs)
                node = self.csi_manager.get_node(self.all_macs[self.active_mac_index])
                name = node.name if node else "?"
                print(f"[Input] Active node: {name}")

        elif key == pygame.K_r:
            print("[Input] Sending reset command (R)")
            self.serial_thread.send_command("R")

        elif key == pygame.K_i:
            print("[Input] Sending info request (I)")
            self.serial_thread.send_command("I")

        elif key == pygame.K_d:
            self.detection_enabled = not self.detection_enabled
            print(f"[Input] Detection {'enabled' if self.detection_enabled else 'disabled'}")
            if self.detection_enabled and self.detection_proc is None:
                self._start_detection()
            elif not self.detection_enabled and self.detection_proc is not None:
                self._stop_detection()

        elif key == pygame.K_s:
            self._save_screenshot()

        elif pygame.K_1 <= key <= pygame.K_4:
            idx = key - pygame.K_1
            if idx < len(self.all_macs):
                self.active_mac_index = idx
                node = self.csi_manager.get_node(self.all_macs[idx])
                name = node.name if node else "?"
                print(f"[Input] Jumped to node {idx + 1}: {name}")

    def _save_screenshot(self) -> None:
        """Save current camera composite + metadata to disk."""
        self._saved_frame_counter += 1
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        out_dir = Path("csi_screenshots")
        out_dir.mkdir(exist_ok=True)

        if self.camera_composite is not None:
            img_path = out_dir / f"capture_{timestamp}_{self._saved_frame_counter:04d}.png"
            cv2.imwrite(str(img_path), self.camera_composite)
            print(f"[Save] Camera frame saved: {img_path}")

        if self.all_macs:
            mac = self.all_macs[self.active_mac_index]
            frame = self.csi_manager.get_node_latest(mac)
            if frame is not None:
                csi_path = out_dir / f"csi_{timestamp}_{self._saved_frame_counter:04d}.json"
                data = {
                    "timestamp": frame.timestamp,
                    "seq": frame.seq,
                    "mac": frame.mac,
                    "rssi": frame.rssi,
                    "bins": frame.bins,
                    "amplitudes": frame.amplitudes.tolist(),
                    "phases": frame.phases.tolist(),
                }
                with open(csi_path, "w") as fh:
                    json.dump(data, fh, indent=2)
                print(f"[Save] CSI data saved: {csi_path}")

    def run(self) -> None:
        """Main application entry point."""
        print(f"\n{'=' * 60}")
        print(f"  {APP_NAME} v{APP_VERSION}")
        print(f"{'=' * 60}\n")

        self.serial_thread.start()
        cam_ok = self._init_cameras()
        if cam_ok:
            print("[Main] Cameras initialised")
        else:
            print("[Main] No cameras available")

        if self.detection_enabled and self.detector_backend != "none":
            self._start_detection()

        clock = pygame.time.Clock()
        display_fps = 0.0
        fps_update_time = time.time()
        frame_count = 0

        print("[Main] Entering main loop. Press Q or ESC to quit.\n")

        try:
            while self._running:
                dt = clock.tick(DEFAULT_FPS) / 1000.0
                self._frame_counter += 1
                frame_count += 1

                now = time.time()
                if now - fps_update_time >= 1.0:
                    display_fps = frame_count / (now - fps_update_time)
                    frame_count = 0
                    fps_update_time = now

                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        self._running = False
                    elif event.type == pygame.KEYDOWN:
                        self._handle_keydown(event)
                    elif event.type == pygame.VIDEORESIZE:
                        self.renderer.width, self.renderer.height = event.size
                        glViewport(0, 0, event.size[0], event.size[1])
                        glMatrixMode(GL_PROJECTION)
                        glLoadIdentity()
                        glOrtho(0.0, event.size[0], event.size[1], 0.0, -1.0, 1.0)
                        glMatrixMode(GL_MODELVIEW)

                self.camera_composite = self._read_cameras()

                if (self.detection_enabled
                        and self.detection_proc is not None
                        and self.det_frame_queue is not None
                        and self.camera_composite is not None):
                    try:
                        self.det_frame_queue.put_nowait(self.camera_composite.copy())
                    except queue.Full:
                        pass

                if self.detection_enabled:
                    self._poll_detection_results()

                if self.detection_enabled and len(self.tracks) >= 1:
                    self._compute_stereo_depth()

                self.all_macs = self.csi_manager.get_active_nodes(timeout=2.0)
                if not self.all_macs:
                    self.all_macs = self.csi_manager.get_all_nodes()
                if self.active_mac_index >= max(len(self.all_macs), 1):
                    self.active_mac_index = 0

                active_node = None
                if self.all_macs:
                    active_node = self.csi_manager.get_node(
                        self.all_macs[self.active_mac_index]
                    )

                if now - self._last_drop_check >= 5.0:
                    dropped = self.csi_manager.drop_stale(timeout=10.0)
                    if dropped:
                        print(f"[Main] Dropped {dropped} stale node(s)")
                    self._last_drop_check = now

                all_node_names = []
                for mac in self.all_macs:
                    node = self.csi_manager.get_node(mac)
                    if node:
                        all_node_names.append(node.name)
                    else:
                        all_node_names.append(mac[:12])

                self.renderer.render_frame(
                    camera_composite=self.camera_composite,
                    active_node=active_node,
                    active_mac_index=self.active_mac_index,
                    all_node_names=all_node_names,
                    active_count=len(self.all_macs),
                    csi_fps=self.serial_thread.csi_fps,
                    tracks=self.tracks,
                    parallax=self.parallax_px,
                    depth=self.depth_m,
                )

        except KeyboardInterrupt:
            print("\n[Main] Interrupted by user")
        finally:
            self._shutdown()

    def _shutdown(self) -> None:
        """Gracefully shut down all subsystems."""
        print("[Main] Shutting down...")
        self._running = False
        self.serial_thread.stop()
        self._stop_detection()
        self._release_cameras()
        self.renderer.cleanup()
        while not self.servo_queue.empty():
            try:
                self.servo_queue.get_nowait()
            except queue.Empty:
                break
        print("[Main] Goodbye!")


# ============================================================================
# Command Line Interface
# ============================================================================

def build_parser() -> argparse.ArgumentParser:
    """Build and return the argument parser with all CLI options."""
    parser = argparse.ArgumentParser(
        description=f"{APP_NAME} v{APP_VERSION} -- Multi-node CSI OpenGL viewer with stereo camera",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Keyboard Controls:\n"
            "  Q / ESC       Quit\n"
            "  LEFT / RIGHT  Cycle active CSI node\n"
            "  R             Reset CSI sequence (send 'R' to ESP32)\n"
            "  I             Request device info (send 'I' to ESP32)\n"
            "  D             Toggle detection on/off\n"
            "  S             Save screenshot + CSI data\n"
            "  1-4           Jump to specific node\n"
        ),
    )

    parser.add_argument(
        "--port", default="/dev/cu.usbserial-1130",
        help="Serial port for CSI data (default: /dev/cu.usbserial-1130)",
    )
    parser.add_argument(
        "--baud", type=int, default=DEFAULT_BAUD,
        help=f"Serial baud rate (default: {DEFAULT_BAUD})",
    )
    parser.add_argument(
        "--webcam", type=int, default=0,
        help="Primary webcam index, used as left camera (default: 0)",
    )
    parser.add_argument(
        "--stereo-right", type=int, default=1,
        help="Right camera device index (default: 1)",
    )
    parser.add_argument(
        "--stereo-left", type=int, default=0,
        help="Left camera device index (default: 0)",
    )
    parser.add_argument(
        "--stereo-detect", action="store_true",
        help="Enable stereo object detection",
    )
    parser.add_argument(
        "--detector-backend",
        choices=["yolo_deepsort", "hog", "none"],
        default="hog",
        help="Detection backend (default: hog)",
    )
    parser.add_argument(
        "--stereo-detect-interval", type=int, default=1,
        help="Run detection every N frames (default: 1)",
    )
    parser.add_argument(
        "--stereo-max-missed", type=int, default=30,
        help="Drop track after N missed frames (default: 30)",
    )
    parser.add_argument(
        "--csi-bins", type=int, default=64,
        help="Expected number of CSI subcarrier bins (default: 64)",
    )
    parser.add_argument(
        "--mac-registry", default=None,
        help="Path to MAC registry JSON (default: auto-resolve)",
    )
    parser.add_argument(
        "--csi-overlay-x", type=int, default=10,
        help="CSI overlay panel X position (default: 10)",
    )
    parser.add_argument(
        "--csi-overlay-y", type=int, default=10,
        help="CSI overlay panel Y position (default: 10)",
    )

    return parser


# ============================================================================
# Entry Point
# ============================================================================

def main() -> None:
    """Application entry point: parse arguments and run the viewer."""
    parser = build_parser()
    args = parser.parse_args()

    # Validate critical dependencies
    missing = []
    if serial is None:
        missing.append("pyserial")
    if cv2 is None:
        missing.append("opencv-python")
    if pygame is None:
        missing.append("pygame")
    try:
        import OpenGL.GL  # noqa: F401
    except ImportError:
        missing.append("PyOpenGL")

    if missing:
        print("ERROR: Missing required dependencies:")
        for pkg in missing:
            print(f"  - {pkg}")
        print("\nInstall with: pip install " + " ".join(missing))
        sys.exit(1)

    # Override camera indices from --webcam if explicitly set
    if args.webcam != 0 and args.stereo_left == 0:
        args.stereo_left = args.webcam

    viewer = UnifiedViewer(args)
    viewer.run()


if __name__ == "__main__":
    main()
