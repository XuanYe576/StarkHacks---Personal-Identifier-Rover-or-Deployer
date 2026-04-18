# CSI + YOLO Human Detection & Tracking — R&D Plan

## Overview
Develop a WiFi CSI-based human detection and tracking system with optional multi-ESP32 support, fused with YOLO computer vision from a Logitech Brio 105 webcam. The system handles unsynchronized ESP32 phase coherence via 100ms phase smoothing and supports both MUSIC (AoA) and FFT-based trilateration/triangulation with overlapping methods.

## Hardware
- **Camera**: Logitech Brio 105 (USB webcam)
- **Primary WiFi**: ESP32 (single CSI source)
- **Multi-ESP Extension**: 2+ ESP32s for AoA/trilateration (phase incoherent)

## Key Technical Challenges
1. ESP32 CSI extraction and preprocessing
2. Phase incoherence across multiple ESP32s (no shared clock)
3. MUSIC algorithm for AoA estimation
4. FFT-based CSI waveform smoothing for multi-ESP
5. Triangulation + trilateration fusion
6. Sensor fusion: CSI WiFi data + YOLO visual detection
7. Optional CLI flag: `--useMultiESP` to enable multi-ESP mode

---

## Stage 1 — Deep Research (Parallel)
**Skill**: `deep-research-swarm`

### Research Tracks (parallel agents):
1. **CSI_Researcher**: ESP32 CSI capabilities, existing libraries (ESP32-CSI-Tool, etc.), CSI packet structure, phase extraction, noise filtering
2. **MUSIC_AoA_Researcher**: MUSIC algorithm for WiFi AoA, subspace methods, ESP32 antenna array constraints, resolution limits
3. **Multi_ESP_Sync_Researcher**: Phase synchronization without hardware sync, 100ms smoothing windows, time-alignment strategies, drift compensation
4. **Sensor_Fusion_Researcher**: CSI + Camera fusion architectures, Kalman filters for WiFi+Vision tracking, YOLO integration patterns, weight-based fusion
5. **Trilateration_Researcher**: FFT-based distance estimation from CSI, multi-node trilateration, triangulation overlap methods, indoor localization accuracy

**Output**: Validated research briefs for each track, cross-verified findings.

---

## Stage 2 — Code Development (Parallel where possible)
**Skill**: `vibecoding-general-swarm`

### Code Modules:
1. **CSI Core Module** (`csi_core.py`): ESP32 CSI data acquisition, parsing, phase extraction, Hampel filter, amplitude/phase separation
2. **MUSIC AoA Module** (`music_aoa.py`): Covariance estimation, eigen-decomposition, MUSIC spectrum, peak detection for angle estimation
3. **Multi-ESP FFT & Phase Smoothing Module** (`multi_esp_fft.py`): 100ms phase smoothing window, FFT per ESP32, phase unwrapping, time-offset compensation
4. **Localization Engine** (`localization.py`): Triangulation (angle intersection) + trilateration (distance circles), weighted overlap fusion, coordinate output
5. **YOLO Detection Tracker** (`yolo_tracker.py`): YOLOv8 inference on Brio 105, bounding box tracking, person detection confidence
6. **Sensor Fusion Core** (`sensor_fusion.py`): Kalman filter fusion of CSI localization + YOLO bounding boxes, dynamic weight assignment
7. **Main Orchestrator** (`main.py`): CLI with `--useMultiESP` flag, pipeline orchestration, real-time visualization
8. **ESP32 Firmware** (`firmware/`): Arduino/PlatformIO firmware for CSI packet streaming over UDP

**Output**: Complete Python codebase with README, requirements.txt, and ESP32 firmware.

---

## Stage 3 — Integration & Testing
- Integrate all modules
- Test with simulated CSI data
- Provide deployment instructions

## Deliverables
- `/mnt/agents/output/csi-tracking-system/` — full codebase
- Research summary document
- ESP32 firmware
- Usage documentation
