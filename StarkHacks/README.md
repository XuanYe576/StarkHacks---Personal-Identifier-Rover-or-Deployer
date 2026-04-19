# StarkHacks ESP32 Control + CSI

This firmware does two things at once:
- Servo control over USB serial: `Servo 1 = elevation`, `Servo 2 = azimuth`
- Wi-Fi CSI capture on ESP32-S3, streamed over USB serial

## Firmware behavior

- Serial speed: `921600`
- Servo command format:
  - `E<angle> A<angle>` (example: `E110 A45`)
- CSI stream line format (current, versioned):
  - `CSIv1,<seq>,<rssi>,<raw_len>,<bins>,A,<amp_0>,...,<amp_(bins-1)>,P,<phase_0>,...,<phase_(bins-1)>`
  - `amp_i`: amplitude from CSI IQ pair `sqrt(I^2 + Q^2)`, quantized to `0..255`
  - `phase_i`: phase from CSI IQ pair `atan2(I, Q)` in centi-degrees (`-18000..18000`)
- Legacy CSI format still accepted by the viewer:
  - `CSI,<seq>,<rssi>,<raw_len>,<bins>,<mag_0>,...,<mag_n>`

## About full I/Q over UART

- Current stream is `amp + phase` quantized.
- If you need full complex CSI, stream raw `I,Q` pairs per subcarrier (`I_0,Q_0,...`) instead of only `amp/phase`.
- Missing bins should be explicit (for example `I=0,Q=0`) but this is lower quality than true captured bins.
- UART can carry full I/Q at this baud, but payload gets larger and packet loss risk increases; use sequence numbers (already present) and drop detection on host.

## Wi-Fi credentials

Set in `platformio.ini` build flags:

```ini
build_flags =
  -DWIFI_SSID=\"YourSSID\"
  -DWIFI_PASSWORD=\"YourPassword\"
```

Optional host/UDP IQ stream flags:

```ini
build_flags =
  -DWIFI_SSID=\"YourSSID\"
  -DWIFI_PASSWORD=\"YourPassword\"
  -DDEVICE_HOSTNAME=\"tzarium-csi-a\"
  -DCSI_UDP_ENABLE=1
  -DCSI_UDP_PORT=3333
  -DCSI_UDP_TARGET=\"255.255.255.255\"
  -DCSI_UDP_IQ_STREAM=1
```

UDP IQ output format:

`CSIIQv1,<seq>,<rssi>,<raw_len>,<bins>,I,<i_0>...,<i_n>,Q,<q_0>...,<q_n>`

## Upload CSI firmware to ESP32

From repo root:

```bash
cd StarkHacks
pio run -t upload --upload-port /dev/cu.usbserial-XXXX
pio device monitor -p /dev/cu.usbserial-XXXX -b 921600
```

Replace `/dev/cu.usbserial-XXXX` with your current device.

## Turn on WiFi UDP + find/listen on macOS

1. Enable `CSI_UDP_ENABLE=1` in `platformio.ini` and upload.
2. Confirm ESP32 prints `UDP CSI target: ...` on serial monitor.
3. On Mac, listen for UDP CSI packets:

```bash
python - <<'PY'
import socket
s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
s.bind(('0.0.0.0',3333))
print('listening udp 3333...')
while True:
    d,a=s.recvfrom(4096)
    print(a, d[:140].decode('utf-8','ignore'))
PY
```

If using unicast target instead of broadcast, discover ESP32 IP by:

```bash
arp -a | grep -Ei 'esp|espressif|192\\.168'
```

## OpenCV OpenGL CSI viewer

Install dependencies in your venv:

```bash
python -m pip install -r tools/requirements-csi-viewer.txt
```

Run (CSI only):

```bash
python tools/csi_viewer_opengl.py --port /dev/cu.usbserial-XXXX --baud 921600 --csi-gaussian-overlay
```

Run (camera + CSI overlays):

```bash
python tools/csi_viewer_opengl.py --port /dev/cu.usbserial-XXXX --baud 921600 --webcam 0
```

Current viewer includes:
- raw amplitude wave (`red`)
- FFT wave (`yellow`)
- Gaussian heat overlay from CSI strength
- optional antenna-POV Gaussian overlay

## WiFiCam model integration

- `third_party/wificam` contains training/inference code and checkpoints.
- This runs on the computer (PyTorch), not on ESP32 directly.
- Directly using pretrained WiFiCam with current ESP32 UART format may work only coarsely unless preprocessing/training format is aligned.

### Live deploy from ESP32 CSI UART

Deployment-style runner (ESP32 CSI serial as direct input):

```bash
python tools/wificam_live_serial.py --port /dev/cu.usbserial-XXXX --baud 921600
```

Dual-node (recommended for your setup `120 + 130`):

```bash
python tools/wificam_live_serial.py \
  --port /dev/cu.usbserial-120 --baud 921600 \
  --port-b /dev/cu.usbserial-130 --baud-b 921600 \
  --fusion mean \
  --checkpoint third_party/wificam/runs/mopoevae_ct/bestLoss.ckpt \
  --device cpu
```

Notes:
- If `third_party/wificam/runs/mopoevae_ct/bestLoss.ckpt` + torch stack are present, it runs live WiFiCam reconstruction.
- If model/torch is missing, it still runs in heatmap-only mode (no crash).
- `--fusion` supports `mean`, `diff`, `interleave`.
- Press `q` to quit.

## Vision-language model (VLM)

- Yes, you can integrate a VLM on the computer side.
- Recommended architecture:
  - ESP32 -> CSI UART stream
  - viewer/model process -> heatmap or reconstruction image
  - VLM process -> consumes image + prompt for semantic output
- Do not run VLM on ESP32; it is host-side only.

### Rescue-oriented VLM return format

For rescue use-cases, force structured JSON output so your controller can parse it safely.

Expected JSON schema:

```json
{
  "scene_summary": "short plain-language summary",
  "risk_level": "low|medium|high|critical",
  "hazards": {
    "smoke_detected": false,
    "fire_visible": false,
    "low_visibility": false
  },
  "human_detected": true,
  "human_count_est": 1,
  "ppe_observed": {
    "glasses_or_goggles": false,
    "mask_or_respirator": false,
    "helmet": false
  },
  "possible_overwall_presence": true,
  "recommended_action": "continue_scan|approach_cautiously|hold_position|request_human_operator|trigger_rescue_protocol",
  "confidence": 0.0,
  "evidence": [
    "why this decision was made"
  ]
}
```

Notes:
- Keep `confidence` in `[0,1]`.
- If uncertain, choose safer action (`request_human_operator` or `hold_position`).
- If smoke/low-visibility cues are present, bias toward conservative actions.
- You can use `tools/vlm_rescue_prompt.txt` as the system/task prompt template.

## WiFi + YOLO + IMU sensor fusion

`tools/csi_viewer_opengl.py` now supports association between:
- WiFi/CSI direction estimate
- YOLO target bearing in camera FOV
- IMU yaw/pitch/roll (from ESP32 IMU serial stream)

New options:
- `--fusion-enable`
- `--imu-serial-port`, `--imu-serial-baud`
- `--imu-yaw-offset-deg`
- `--wifi-look-yaw-deg`
- `--fusion-assoc-gate-deg`
- `--fusion-csi-min`

Example:

```bash
python tools/csi_viewer_opengl.py \
  --port /dev/cu.usbserial-1120 --baud 921600 \
  --webcam 0 --stereo-right 1 --stereo-detect \
  --detector-backend yolo_deepsort \
  --fusion-enable \
  --imu-serial-port /dev/cu.usbserial-IMU --imu-serial-baud 115200 \
  --imu-yaw-offset-deg 0 \
  --wifi-look-yaw-deg 0 \
  --fusion-assoc-gate-deg 18 \
  --fusion-csi-min 0.20
```

Accepted IMU serial line formats:
- `IMU,yaw,pitch,roll`
- `YPR,yaw,pitch,roll`
- `{\"yaw\":12.3,\"pitch\":-1.0,\"roll\":0.2}`
- `yaw=12.3 pitch=-1.0 roll=0.2`

## Optional multi-ESP trilateration (coarse)

`tools/csi_viewer_opengl.py` also supports coarse 2D trilateration from 3 CSI nodes (A/B/C) using RSSI-distance model:

```bash
python tools/csi_viewer_opengl.py \
  --port /dev/cu.usbserial-120 --baud 921600 \
  --port-b /dev/cu.usbserial-130 --baud-b 921600 \
  --port-c /dev/cu.usbserial-140 --baud-c 921600 \
  --trilateration-enable \
  --anchor-a 0.0,0.0 --anchor-b 1.0,0.0 --anchor-c 0.0,1.0 \
  --rssi-ref-db -40 --path-loss-exp 2.2
```

Notes:
- This is RSSI-based and coarse (sensitive to environment).
- For best results, place anchors with known spacing and keep units in meters.
- This is not equivalent to ESPARGOS coherent-array MUSIC maps.
- ESPARGOS/MUSIC-style heatmaps require phase-coherent multi-antenna array data.

## Single AP A/B relay (B -> A -> PC)

Use this when Mac can only connect to one Wi-Fi:
- Rover runs AP: `TzariumRover`
- CSI node A joins Rover AP as STA and runs aggregator role
- CSI node B joins Rover AP as STA and sends IQ only to A
- A fuses local+remote IQ and sends one CSI UDP stream to PC

Role flags for `StarkHacks/src/main.cpp`:

- Node A (aggregator):
  - `-DCSI_WIFI_AP_MODE=0`
  - `-DWIFI_SSID=\"TzariumRover\"`
  - `-DWIFI_PASSWORD=\"rover1234\"`
  - `-DCSI_NODE_ROLE=1`
  - `-DCSI_RELAY_LISTEN_PORT=3340`
  - `-DCSI_UDP_TARGET=\"255.255.255.255\"`
  - `-DCSI_UDP_PORT=3333`

- Node B (child):
  - `-DCSI_WIFI_AP_MODE=0`
  - `-DWIFI_SSID=\"TzariumRover\"`
  - `-DWIFI_PASSWORD=\"rover1234\"`
  - `-DCSI_NODE_ROLE=2`
  - `-DCSI_RELAY_TARGET=\"192.168.4.2\"` (replace with A's IP)
  - `-DCSI_RELAY_LISTEN_PORT=3340`

Notes:
- `CSI_NODE_ROLE=1` fuses A/B by per-bin I/Q average.
- If B is missing for ~300 ms, A falls back to local CSI only.

## Webcam note

- Logitech Brio 105 connected by USB to the computer.
