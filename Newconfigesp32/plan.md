# Plan: Add MAC Address-Based ESP32 Identification to UART CSI Pipeline

## Context
The user has a `StarkHacks` system with ESP32 sending CSI data over UART in `CSIv1` format, a unified OpenGL Python viewer, and YOLO/DeepSORT stereo detection. They currently have 1 ESP32 node and plan to add 3 more (total 4). They need MAC address-based identification added so each node's data can be distinguished.

## Current System
- **ESP32 firmware**: `StarkHacks/src/main.cpp` - outputs CSI as `CSIv1,<seq>,<rssi>,<raw_len>,<bins>,A,<amp...>,P,<phase...>`
- **Viewer**: `StarkHacks/tools/csi_viewer_opengl.py` - parses CSI, displays stereo cameras + CSI overlay
- **Servo control**: Serial commands `E<angle> A<angle>`

## Stages

### Stage 1 — Read & Understand Current Code
Read the ESP32 firmware and viewer Python to understand current CSI format, parsing logic, and display pipeline.

**Files to read:**
- `StarkHacks/src/main.cpp`
- `StarkHacks/tools/csi_viewer_opengl.py`

### Stage 2 — Firmware Changes (ESP32 C++)
Add MAC address to CSI output. Update the `CSIv1` format to include sender MAC address.

**New format**: `CSIv1,<seq>,<rssi>,<raw_len>,<bins>,<macaddr>,A,<amp...>,P,<phase...>`

### Stage 3 — Viewer Changes (Python)
- Parse MAC address field in CSI lines
- Add MAC-to-label mapping (e.g., friendly names per MAC)
- Display which ESP32 node the current CSI frame is from
- Add multi-node CSI buffer management (prep for 4 nodes)
- Display per-node CSI stats in overlay

### Stage 4 — Integration & Validation
- Ensure firmware + viewer compatibility
- Add backward compatibility (handle frames without MAC for transition)
- Document the updated protocol

## Skills Used
- `vibecoding-general-swarm` for general coding orchestration
