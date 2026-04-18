"""Configuration management for the CSI + YOLO Human Tracking System.

This module defines strongly-typed configuration dataclasses for each
sub-system (CSI acquisition, YOLO detection/tracking, and sensor fusion)
and provides YAML-based configuration loading.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Tuple

try:
    import yaml
except ImportError as exc:  # pragma: no cover
    raise ImportError(
        "PyYAML is required for configuration loading. "
        "Install it with: pip install pyyaml"
    ) from exc

logger = logging.getLogger(__name__)


# --------------------------------------------------------------------------- #
# Defaults
# --------------------------------------------------------------------------- #

def _default_esp32_ip() -> List[str]:
    """Factory so each dataclass instance gets its own list."""
    return ["192.168.1.100"]


def _default_yolo_classes() -> List[int]:
    """Factory so each dataclass instance gets its own list."""
    return [0]  # COCO class 0 == person


# --------------------------------------------------------------------------- #
# Configuration dataclasses
# --------------------------------------------------------------------------- #

@dataclass
class CSIConfig:
    """Configuration for ESP32 CSI data acquisition.

    Attributes:
        esp32_ip: List of ESP32 IP addresses (UDP streaming mode).
        esp32_port: UDP port number the ESP32s stream CSI data to.
        sample_rate: Target CSI sample rate in Hz.
        window_size_ms: Phase smoothing window duration in milliseconds.
        subcarrier_count: Number of subcarriers (ESP32 HT20 = 52).
        use_multi_esp: Enable multi-ESP32 mode with phase smoothing.
    """
    esp32_ip: List[str] = field(default_factory=_default_esp32_ip)
    esp32_port: int = 5005
    sample_rate: int = 100
    window_size_ms: int = 100
    subcarrier_count: int = 52
    use_multi_esp: bool = False

    def __post_init__(self) -> None:
        if not self.esp32_ip:
            raise ValueError("esp32_ip must contain at least one IP address")
        if self.sample_rate <= 0:
            raise ValueError("sample_rate must be positive")
        if self.subcarrier_count <= 0:
            raise ValueError("subcarrier_count must be positive")


@dataclass
class YOLOConfig:
    """Configuration for YOLO-based object detection and tracking.

    Attributes:
        model: Path or Ultralytics model identifier (e.g. ``yolo11n.pt``).
        conf_thresh: Minimum confidence for a detection to be accepted.
        classes: List of COCO class IDs to detect (``[0]`` = person only).
        track_buffer: ByteTrack buffer length (frames).
        camera_id: OpenCV camera device index (0 = first USB camera).
        resolution: Desired capture resolution as ``(width, height)``.
    """
    model: str = "yolo11n.pt"
    conf_thresh: float = 0.5
    classes: List[int] = field(default_factory=_default_yolo_classes)
    track_buffer: int = 30
    camera_id: int = 0
    resolution: Tuple[int, int] = (640, 480)

    def __post_init__(self) -> None:
        if not self.model:
            raise ValueError("model must be a non-empty string")
        if not 0.0 <= self.conf_thresh <= 1.0:
            raise ValueError("conf_thresh must be in [0.0, 1.0]")
        if self.track_buffer <= 0:
            raise ValueError("track_buffer must be positive")
        if len(self.resolution) != 2 or any(r <= 0 for r in self.resolution):
            raise ValueError("resolution must be a positive (width, height) tuple")


@dataclass
class FusionConfig:
    """Configuration for EKF-based sensor fusion of WiFi CSI and YOLO data.

    Attributes:
        process_noise: EKF process noise covariance (Q).
        measurement_noise_wifi: EKF measurement noise for WiFi positions (R).
        measurement_noise_cam: EKF measurement noise for camera positions (R).
        fusion_weight_wifi: Static fallback weight for WiFi measurements.
        fusion_weight_cam: Static fallback weight for camera measurements.
    """
    process_noise: float = 0.1
    measurement_noise_wifi: float = 2.0
    measurement_noise_cam: float = 0.3
    fusion_weight_wifi: float = 0.3
    fusion_weight_cam: float = 0.7

    def __post_init__(self) -> None:
        if self.process_noise < 0:
            raise ValueError("process_noise must be non-negative")
        if self.measurement_noise_wifi < 0:
            raise ValueError("measurement_noise_wifi must be non-negative")
        if self.measurement_noise_cam < 0:
            raise ValueError("measurement_noise_cam must be non-negative")
        total_weight = self.fusion_weight_wifi + self.fusion_weight_cam
        if not (0.99 <= total_weight <= 1.01):
            raise ValueError(
                f"fusion weights must sum to 1.0, got {total_weight}"
            )


# --------------------------------------------------------------------------- #
# YAML helpers
# --------------------------------------------------------------------------- #

def _flatten_yaml(raw: Dict[str, Any]) -> Dict[str, Any]:
    """Flatten a potentially nested YAML dict into a single-level dict.

    Nested keys are joined with ``.``, e.g. ``csi.esp32_ip`` becomes
    ``csi_esp32_ip`` — but we prefer matching the exact dataclass field
    names, so we also check both nested and flat forms.
    """
    flat: Dict[str, Any] = {}
    for key, value in raw.items():
        if isinstance(value, dict):
            for sub_key, sub_value in value.items():
                flat[f"{key}_{sub_key}"] = sub_value
                flat[sub_key] = sub_value  # also keep bare name
        else:
            flat[key] = value
    return flat


def _map_to_dataclass_params(
    flat: Dict[str, Any],
    dataclass_type: type,
) -> Dict[str, Any]:
    """Extract parameters from *flat* that match fields of *dataclass_type*."""
    field_names = {f.name for f in dataclass_type.__dataclass_fields__.values()}
    return {k: v for k, v in flat.items() if k in field_names}


# --------------------------------------------------------------------------- #
# Public API
# --------------------------------------------------------------------------- #

def load_config(path: str) -> Tuple[CSIConfig, YOLOConfig, FusionConfig]:
    """Load and validate a YAML configuration file.

    The YAML file may be flat or nested.  Expected top-level keys are
    ``csi``, ``yolo``, and ``fusion``, but any structure is accepted as
    long as the individual field names can be matched.

    Args:
        path: Filesystem path to the YAML configuration file.

    Returns:
        A tuple ``(csi_config, yolo_config, fusion_config)``.

    Raises:
        FileNotFoundError: If *path* does not exist.
        ValueError: If the YAML is malformed or a config is invalid.
    """
    p = Path(path)
    if not p.exists():
        raise FileNotFoundError(f"Configuration file not found: {path}")

    with p.open("r", encoding="utf-8") as fh:
        raw: Dict[str, Any] = yaml.safe_load(fh) or {}

    flat = _flatten_yaml(raw)

    csi_params = _map_to_dataclass_params(flat, CSIConfig)
    yolo_params = _map_to_dataclass_params(flat, YOLOConfig)
    fusion_params = _map_to_dataclass_params(flat, FusionConfig)

    # Handle the common nested form: keys under 'csi', 'yolo', 'fusion'
    section_types = {
        "csi": CSIConfig,
        "yolo": YOLOConfig,
        "fusion": FusionConfig,
    }
    for section, target in (
        ("csi", csi_params),
        ("yolo", yolo_params),
        ("fusion", fusion_params),
    ):
        if section in raw and isinstance(raw[section], dict):
            dc_type = section_types[section]
            for key, value in raw[section].items():
                if key in dc_type.__dataclass_fields__:
                    target.setdefault(key, value)

    csi_cfg = CSIConfig(**csi_params)
    yolo_cfg = YOLOConfig(**yolo_params)
    fusion_cfg = FusionConfig(**fusion_params)

    logger.info(
        "Loaded config from %s — ESP32s: %s, model: %s, fusion weights: %.1f/%.1f",
        p,
        csi_cfg.esp32_ip,
        yolo_cfg.model,
        fusion_cfg.fusion_weight_wifi,
        fusion_cfg.fusion_weight_cam,
    )
