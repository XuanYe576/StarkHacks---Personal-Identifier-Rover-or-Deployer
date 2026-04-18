#!/usr/bin/env python3
"""
CSI + YOLO Human Detection & Tracking System

Usage:
    python main.py                          # Single ESP32 mode
    python main.py --useMultiESP            # Multi-ESP32 mode with phase smoothing
    python main.py --esp-ips 192.168.1.100 192.168.1.101 --useMultiESP

This script is the CLI entry point for the real-time human tracking
pipeline that fuses WiFi CSI measurements from ESP32(s) with YOLO11
computer-vision detections from a Logitech Brio 105 webcam.

Press Ctrl+C to shut down gracefully.
"""

from __future__ import annotations

import argparse
import logging
import signal
import sys
import time
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

# ---------------------------------------------------------------------------
# Configure logging early so import-time messages are visible
# ---------------------------------------------------------------------------
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s"
)
logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Attempt to import project modules (with graceful fallback for missing deps)
# ---------------------------------------------------------------------------

try:
    from src.config import CSIConfig, YOLOConfig, FusionConfig, load_config
    _HAS_CONFIG = True
except Exception as exc:
    logger.warning("config.py not available (%s); using inline dataclasses.", exc)
    _HAS_CONFIG = False

try:
    from src.pipeline import TrackingPipeline, PipelineVisualizer
    _HAS_PIPELINE = True
except Exception as exc:
    logger.error("pipeline.py not available (%s). Cannot start pipeline.", exc)
    _HAS_PIPELINE = False


# ---------------------------------------------------------------------------
# Inline dataclass fallbacks (used when config.py is absent)
# ---------------------------------------------------------------------------

if not _HAS_CONFIG:
    from dataclasses import dataclass, field
    from typing import List, Tuple

    @dataclass
    class CSIConfig:  # type: ignore[no-redef]
        """CSI source configuration."""
        esp32_ip: List[str] = field(default_factory=lambda: ["192.168.1.100"])
        esp32_port: int = 5005
        sample_rate: int = 100
        window_size_ms: int = 100
        subcarrier_count: int = 52
        use_multi_esp: bool = False

    @dataclass
    class YOLOConfig:  # type: ignore[no-redef]
        """YOLO tracker configuration."""
        model: str = "yolo11n.pt"
        conf_thresh: float = 0.5
        classes: Tuple[int, ...] = (0,)
        track_buffer: int = 30
        camera_id: int = 0
        resolution: Tuple[int, int] = (640, 480)

    @dataclass
    class FusionConfig:  # type: ignore[no-redef]
        """EKF fusion configuration."""
        process_noise: float = 0.1
        measurement_noise_wifi: float = 2.0
        measurement_noise_cam: float = 0.3
        fusion_weight_wifi: float = 0.3
        fusion_weight_cam: float = 0.7

    def load_config(path: str) -> Tuple[Any, Any, Any]:  # type: ignore[no-redef]
        """Stub: load configuration from a YAML file.

        In a full deployment this would parse a YAML config file and
        return populated dataclass instances.

        Args:
            path: Path to YAML configuration file.

        Returns:
            Tuple of (CSIConfig, YOLOConfig, FusionConfig).

        Raises:
            NotImplementedError: Always, because PyYAML may not be installed.
        """
        raise NotImplementedError(
            "YAML config loading requires config.py with PyYAML support. "
            f"File path was: {path}"
        )


# ---------------------------------------------------------------------------
# Signal handler for graceful shutdown
# ---------------------------------------------------------------------------

_running = True


def signal_handler(sig: int, frame: Any) -> None:
    """Handle Ctrl+C (SIGINT) for clean shutdown."""
    global _running
    print("\nShutting down...")
    _running = False


signal.signal(signal.SIGINT, signal_handler)


# ---------------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------------

def build_argument_parser() -> argparse.ArgumentParser:
    """Build and return the argument parser for the CLI."""
    parser = argparse.ArgumentParser(
        description="CSI + YOLO Human Tracking System",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s                              # Single ESP32, default settings
  %(prog)s --useMultiESP                # Multi-ESP32 with phase smoothing
  %(prog)s --esp-ips 192.168.1.100 192.168.1.101 --useMultiESP --visualize
  %(prog)s --camera 1 --config config.yaml
        """.strip(),
    )
    parser.add_argument(
        "--useMultiESP",
        action="store_true",
        help=(
            "Enable multi-ESP32 mode with FFT trilateration + 100ms phase smoothing. "
            "When set, the system treats each IP in --esp-ips as a separate ESP32 "
            "CSI source and runs the multi-ESP FFT localisation pipeline."
        ),
    )
    parser.add_argument(
        "--esp-ips",
        nargs="+",
        default=["192.168.1.100"],
        help="ESP32 IP addresses (space-separated). Defaults to 192.168.1.100.",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=5005,
        help="UDP port for CSI streaming (default: 5005).",
    )
    parser.add_argument(
        "--camera",
        type=int,
        default=0,
        help="Camera device ID (default: 0). Use 1, 2, etc. for secondary cameras.",
    )
    parser.add_argument(
        "--visualize",
        action="store_true",
        help="Show real-time matplotlib visualisation window (requires display).",
    )
    parser.add_argument(
        "--config",
        type=str,
        default=None,
        help="Path to YAML configuration file. If provided, CLI flags override YAML values.",
    )
    parser.add_argument(
        "--log-level",
        type=str,
        default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        help="Logging verbosity (default: INFO).",
    )
    return parser


def create_configs(args: argparse.Namespace) -> Tuple[Any, Any, Any]:
    """Create configuration objects from CLI arguments.

    If ``--config`` is provided, load from YAML and selectively override
    with CLI flags.  Otherwise, build configs purely from CLI values.

    Args:
        args: Parsed argument namespace.

    Returns:
        Tuple of (CSIConfig, YOLOConfig, FusionConfig).
    """
    if args.config is not None:
        logger.info("Loading configuration from %s ...", args.config)
        csi_cfg, yolo_cfg, fusion_cfg = load_config(args.config)

        # CLI overrides
        csi_cfg.esp32_ip = args.esp_ips
        csi_cfg.esp32_port = args.port
        csi_cfg.use_multi_esp = args.useMultiESP
        yolo_cfg.camera_id = args.camera
    else:
        csi_cfg = CSIConfig(
            esp32_ip=args.esp_ips,
            esp32_port=args.port,
            use_multi_esp=args.useMultiESP,
        )
        yolo_cfg = YOLOConfig(camera_id=args.camera)
        fusion_cfg = FusionConfig()

    return csi_cfg, yolo_cfg, fusion_cfg


def print_banner(csi_cfg: Any, yolo_cfg: Any, fusion_cfg: Any) -> None:
    """Print a startup banner showing the active configuration."""
    mode_str = "Multi-ESP" if csi_cfg.use_multi_esp else "Single-ESP"
    esp_str = ", ".join(csi_cfg.esp32_ip) if isinstance(csi_cfg.esp32_ip, list) else str(csi_cfg.esp32_ip)

    banner = f"""
{'=' * 60}
  CSI + YOLO Human Detection & Tracking System
{'=' * 60}
  Mode           : {mode_str}
  ESP32 IPs      : {esp_str}
  UDP Port       : {csi_cfg.esp32_port}
  Camera ID      : {yolo_cfg.camera_id}
  YOLO Model     : {yolo_cfg.model}
  WiFi Noise (R) : {fusion_cfg.measurement_noise_wifi}
  Cam Noise (R)  : {fusion_cfg.measurement_noise_cam}
{'=' * 60}
"""
    print(banner)


def main() -> int:
    """Main entry point for the CSI + YOLO tracking application.

    Returns:
        Exit code (0 for clean exit, 1 for error).
    """
    parser = build_argument_parser()
    args = parser.parse_args()

    # Adjust logging level
    logging.getLogger().setLevel(getattr(logging, args.log_level.upper()))

    if not _HAS_PIPELINE:
        logger.error("Cannot start: pipeline module is not available.")
        return 1

    # ------------------------------------------------------------------
    # Create configurations
    # ------------------------------------------------------------------
    try:
        csi_cfg, yolo_cfg, fusion_cfg = create_configs(args)
    except Exception as exc:
        logger.error("Failed to create configuration: %s", exc)
        return 1

    print_banner(csi_cfg, yolo_cfg, fusion_cfg)

    # ------------------------------------------------------------------
    # Initialise and start pipeline
    # ------------------------------------------------------------------
    try:
        pipeline = TrackingPipeline(csi_cfg, yolo_cfg, fusion_cfg)
    except Exception as exc:
        logger.error("Failed to initialise pipeline: %s", exc)
        return 1

    pipeline.start()
    logger.info("Pipeline started. Press Ctrl+C to stop.")

    # ------------------------------------------------------------------
    # Optional visualiser
    # ------------------------------------------------------------------
    visualizer: Any = None
    if args.visualize:
        try:
            visualizer = PipelineVisualizer(
                room_bounds=(0.0, 5.0, 0.0, 4.0),
                anchor_positions=None,  # Will be populated from CSI config in future
            )
            logger.info("Visualizer enabled.")
        except Exception as exc:
            logger.warning("Could not initialise visualiser: %s", exc)

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------
    global _running
    frame_count = 0
    start_time = time.monotonic()

    try:
        while _running:
            state = pipeline.get_state()

            if state["position"] is not None:
                px, py = state["position"]
                bx, by = state["wifi_bias"]
                print(
                    f"Position: ({px:6.2f}, {py:6.2f})  "
                    f"WiFi Bias: ({bx:6.2f}, {by:6.2f})  "
                    f"Tracks: {len(state['tracks']):2d}",
                    end="\r",
                    flush=True,
                )

            if visualizer is not None:
                visualizer.update(state)
                img = visualizer.render()
                try:
                    import cv2
                    cv2.imshow("CSI + YOLO Tracking", img)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        _running = False
                except ImportError:
                    pass

            frame_count += 1
            time.sleep(0.033)  # ~30 Hz display loop

    except KeyboardInterrupt:
        logger.info("KeyboardInterrupt received.")
    finally:
        pipeline.stop()

        # Print summary statistics
        elapsed = time.monotonic() - start_time
        if elapsed > 0:
            print(
                f"\n\nProcessed {frame_count} frames in {elapsed:.1f}s "
                f"({frame_count / elapsed:.1f} fps effective)."
            )

        if visualizer is not None:
            try:
                import cv2
                cv2.destroyAllWindows()
            except ImportError:
                pass

        print("Shutdown complete.")

    return 0


# ---------------------------------------------------------------------------
# Self-test (can be run standalone without sibling modules)
# ---------------------------------------------------------------------------

def _self_test() -> int:
    """Run basic sanity checks on argument parsing and config creation.

    Returns:
        Number of failed tests.
    """
    print("=" * 60)
    print("main.py self-test")
    print("=" * 60)

    failures = 0

    # 1. Argument parser build
    try:
        parser = build_argument_parser()
        args = parser.parse_args(["--useMultiESP", "--esp-ips", "10.0.0.1", "10.0.0.2"])
        assert args.useMultiESP is True
        assert args.esp_ips == ["10.0.0.1", "10.0.0.2"]
        assert args.port == 5005
        print("[PASS] Argument parsing (--useMultiESP + custom IPs)")
    except Exception as exc:
        print(f"[FAIL] Argument parsing: {exc}")
        failures += 1

    # 2. Default arguments
    try:
        parser2 = build_argument_parser()
        args2 = parser2.parse_args([])
        assert args2.useMultiESP is False
        assert args2.esp_ips == ["192.168.1.100"]
        assert args2.camera == 0
        assert args2.visualize is False
        print("[PASS] Default argument values")
    except Exception as exc:
        print(f"[FAIL] Default arguments: {exc}")
        failures += 1

    # 3. Config creation (no YAML)
    try:
        parser3 = build_argument_parser()
        args3 = parser3.parse_args(["--useMultiESP", "--esp-ips", "192.168.1.10"])
        csi, yolo, fusion = create_configs(args3)
        assert csi.use_multi_esp is True
        assert csi.esp32_ip == ["192.168.1.10"]
        assert yolo.camera_id == 0
        print("[PASS] Config creation from CLI args")
    except Exception as exc:
        print(f"[FAIL] Config creation: {exc}")
        failures += 1

    # 4. Config creation with camera override
    try:
        parser4 = build_argument_parser()
        args4 = parser4.parse_args(["--camera", "1"])
        _, yolo4, _ = create_configs(args4)
        assert yolo4.camera_id == 1
        print("[PASS] Camera ID override")
    except Exception as exc:
        print(f"[FAIL] Camera override: {exc}")
        failures += 1

    # 5. Module availability checks
    print(f"[INFO] config.py available : {_HAS_CONFIG}")
    print(f"[INFO] pipeline.py available: {_HAS_PIPELINE}")

    print("=" * 60)
    if failures == 0:
        print("All self-tests passed.")
    else:
        print(f"Self-test finished with {failures} failure(s).")
    print("=" * 60)
    return failures


if __name__ == "__main__":
    # Allow: python main.py --self-test
    if "--self-test" in sys.argv:
        sys.exit(_self_test())
    sys.exit(main())
