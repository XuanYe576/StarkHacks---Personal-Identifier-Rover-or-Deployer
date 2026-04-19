# LLM + CV + RF + Multi-Node Instructions

This guide is the operational playbook for the current project stack.

## 1) LLM (Rescue Reasoning Output)

Use the rescue prompt template:
- `StarkHacks/tools/vlm_rescue_prompt.txt`

Expected structured JSON fields (for automation):
- `scene_summary`
- `risk_level`
- `hazards.smoke_detected`
- `hazards.fire_visible`
- `hazards.low_visibility`
- `human_detected`
- `human_count_est`
- `ppe_observed.glasses_or_goggles`
- `ppe_observed.mask_or_respirator`
- `ppe_observed.helmet`
- `possible_overwall_presence`
- `recommended_action`
- `confidence`
- `evidence`

Use strict JSON-only output so rover-side logic can parse results directly.

## 2) CV (Camera + YOLO + DeepSORT)

Run viewer with CSI + stereo + YOLO/DeepSORT:

```bash
cd /Users/zhuowenfeng6626/Downloads/StarkHacks/StarkHacks---Personal-Identifier-Rover-or-Deployer
source .venv-yolo310/bin/activate

python StarkHacks/tools/csi_viewer_opengl.py \
  --port /dev/cu.usbserial-130 --baud 921600 \
  --webcam 0 --stereo-right 1 --stereo-detect \
  --detector-backend yolo_deepsort \
  --stereo-detect-interval 1 --stereo-max-missed 30 \
  --fusion-enable --fusion-assoc-gate-deg 18 --fusion-csi-min 0.20
```

If YOLO backend dependencies are incomplete, temporary fallback:

```bash
--detector-backend hog
```

## 3) RF (CSI Pipeline)

RF source is ESP32 CSI UART/UDP stream.

Current CSI packet format:
- `CSIv1,<seq>,<rssi>,<raw_len>,<bins>,A,<amp...>,P,<phase...>`
- UDP IQ format:
  - `CSIIQv1,<seq>,<rssi>,<raw_len>,<bins>,I,<i...>,Q,<q...>`

Core CSI firmware:
- `StarkHacks/src/main.cpp`

Key role flags:
- `CSI_NODE_ROLE=0` direct-to-PC
- `CSI_NODE_ROLE=1` aggregator (A)
- `CSI_NODE_ROLE=2` child (B -> A relay)

## 4) Multi-Node Modes

### Mode A: 2-board practical setup (current most common)
- Board 1: Rover AP/controller
- Board 2: CSI node (single CSI stream)
- Result: single-node CSI + rover control

### Mode B: 3-board full relay/fusion setup
- Board 1: Rover AP/controller
- Board 2: CSI node A (`CSI_NODE_ROLE=1`)
- Board 3: CSI node B (`CSI_NODE_ROLE=2`, sends to A)
- Result: B sends IQ to A, A fuses and forwards to PC

If Mac can only join one Wi-Fi, use Rover AP (`TzariumRover`) as network center.

## 5) Known Operational Notes

- If upload says serial port busy, close monitor first (`Ctrl+C`).
- Do not run CSI upload while that same port is open in monitor/viewer.
- If WiFiCam says model load failed and switches to heatmap-only mode, install missing Python deps in the active venv.
- If only one USB serial device appears, you currently only have one connected ESP32.

## 6) Quick Commands

Single-node WiFiCam heatmap (safe baseline):

```bash
cd /Users/zhuowenfeng6626/Downloads/StarkHacks/StarkHacks---Personal-Identifier-Rover-or-Deployer/StarkHacks
source /Users/zhuowenfeng6626/Downloads/StarkHacks/StarkHacks---Personal-Identifier-Rover-or-Deployer/.venv-wificam310/bin/activate
python tools/wificam_live_serial.py --port /dev/cu.usbserial-130 --baud 921600 --disable-model
```

Rover UDP test packet:

```bash
python3 - <<'PY'
import socket, time
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
addr = ("192.168.4.1", 7001)
s.sendto(b"0.6,0,0,95,1", addr)  # forward
time.sleep(1.0)
s.sendto(b"0,0,0,95,0", addr)    # stop
print("done")
PY
```
