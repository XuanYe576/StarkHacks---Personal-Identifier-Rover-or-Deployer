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

## Wi-Fi credentials

Set in `platformio.ini` build flags:

```ini
build_flags =
  -DWIFI_SSID=\"YourSSID\"
  -DWIFI_PASSWORD=\"YourPassword\"
```

## OpenCV OpenGL CSI viewer

Install dependencies:

```bash
python3 -m pip install -r tools/requirements-csi-viewer.txt
```

Run:

```bash
python3 tools/csi_viewer_opengl.py --port /dev/ttyACM0 --baud 921600
```

This viewer shows two CSI panels:
- Top: amplitude waterfall
- Bottom: phase waterfall

With webcam in same OpenGL window:

```bash
python3 tools/csi_viewer_opengl.py --port /dev/ttyACM0 --webcam /dev/v4l/by-id/<your-device-id>
```

You can also pass webcam index (for example `--webcam 0`).

## Webcam note

Recorded hardware note:
- Logitech Brio 105 webcam connected by USB directly to the computer.
