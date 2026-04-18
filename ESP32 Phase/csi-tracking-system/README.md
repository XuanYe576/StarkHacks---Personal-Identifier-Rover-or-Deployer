# CSI Tracking System

[![Python 3.10+](https://img.shields.io/badge/python-3.10+-blue.svg)](https://www.python.org/downloads/)
[![NumPy](https://img.shields.io/badge/numpy-1.24+-green.svg)](https://numpy.org/)
[![ESP32](https://img.shields.io/badge/ESP32-Compatible-orange.svg)](https://www.espressif.com/)
[![License](https://img.shields.io/badge/license-MIT-purple.svg)](LICENSE)

A real-time human tracking system using **WiFi Channel State Information (CSI)** from ESP32 microcontrollers combined with computer vision for person detection and localization.

---

## Table of Contents

1. [System Overview](#system-overview)
2. [Architecture](#architecture)
3. [Hardware Requirements](#hardware-requirements)
4. [Installation](#installation)
5. [Firmware Setup](#firmware-setup)
6. [Usage](#usage)
7. [Configuration](#configuration)
8. [Module Descriptions](#module-descriptions)
9. [API Reference](#api-reference)
10. [Troubleshooting](#troubleshooting)
11. [Research & Citations](#research--citations)
12. [License](#license)

---

## System Overview

The CSI Tracking System combines WiFi-based sensing with computer vision to achieve robust indoor human tracking. It leverages the ESP32's ability to extract Channel State Information (CSI) from WiFi packets, which captures the fine-grained physical layer information about how wireless signals propagate through the environment. When a person moves through a space, their body perturbs the WiFi signal, causing detectable changes in the CSI that can be used to estimate their position.

### Key Features

- **Real-time CSI streaming** at 100 Hz from ESP32 to Python server via UDP
- **Multi-ESP32 triangulation** support for improved localization accuracy
- **Computer vision fusion** using YOLO and Kalman filters for robust tracking
- **Signal processing pipeline** with noise filtering, phase sanitization, and subcarrier selection
- **Visualization tools** for CSI heatmaps, tracking trajectories, and signal analysis
- **Configurable YAML-based** system configuration

---

## Architecture

```
                    +-----------------------+
                    |   Python Server       |
                    |   (Processing Hub)    |
                    |  192.168.1.200:5005  |
                    +-----------+-----------+
                                |
                    +-----------v-----------+
                    |   UDP Data Receiver   |
                    |   (Binary Parser)     |
                    +-----------+-----------+
                                |
              +-----------------+-----------------+
              |                                   |
+-------------v-------------+   +-----------------v-------------+
|   ESP32 #1 (Transmitter)  |   |   ESP32 #2 (Transmitter)      |
|   192.168.1.100           |   |   192.168.1.101               |
|                           |   |                               |
|  [WiFi CSI] --> [UDP]     |   |  [WiFi CSI] --> [UDP]         |
+-------------+-------------+   +-----------------+-------------+
              |                                   |
              +-----------------+-----------------+
                                |
                    +-----------v-----------+
                    |   Signal Processing   |
                    |   - Noise filtering   |
                    |   - Phase unwrap      |
                    |   - Subcarrier select |
                    +-----------+-----------+
                                |
              +-----------------+-----------------+
              |                                   |
+-------------v-------------+   +-----------------v-------------+
|   Motion Detection        |   |   Computer Vision             |
|   - Hampel filter         |   |   - YOLO object detection     |
|   - Variance analysis     |   |   - Kalman filter tracking    |
|   - Activity classification|   |   - Person detection          |
+-------------+-------------+   +-----------------+-------------+
              |                                   |
              +-----------------+-----------------+
                                |
                    +-----------v-----------+
                    |   Fusion Engine       |
                    |   - Sensor fusion      |
                    |   - Position estimate  |
                    |   - Trajectory smooth  |
                    +-----------+-----------+
                                |
                    +-----------v-----------+
                    |   Visualization       |
                    |   - Real-time plots    |
                    |   - Trajectory map     |
                    |   - CSI heatmap        |
                    +-----------------------+

Binary Frame Format:
+----------------+-----------+--------+------------+----------+
| Magic (4B)     | Timestamp | RSSI   | CSI Length | CSI Data |
| 0xC5110001     | (8B dbl)  | (4B)   | (4B)       | (N B)    |
+----------------+-----------+--------+------------+----------+
```

---

## Hardware Requirements

### Required Components

| Component | Specification | Purpose | Qty |
|-----------|--------------|---------|-----|
| **ESP32 Development Board** | ESP32-WROOM-32, ESP32-S3, or ESP32-C6 | CSI data collection | 1-3 |
| **USB Cable** | Micro-USB or USB-C | Power and programming | 1-3 |
| **Computer** | Intel i5+ / 8GB RAM | Python server processing | 1 |
| **WiFi Router** | 802.11n/g, 2.4GHz band | Network infrastructure | 1 |

### Optional Components

| Component | Specification | Purpose |
|-----------|--------------|---------|
| **Logitech Brio 105** | 1080p Webcam | Computer vision tracking |
| **USB 3.0 Hub** | Powered | Multiple device connectivity |
| **Breadboard + Jumper Wires** | Standard | Prototype expansions |

### ESP32 Recommended Boards

- **ESP32-WROOM-32**: Most common, well-tested, cost-effective
- **ESP32-S3**: Dual-core 240MHz, AI/vector instructions, faster processing
- **ESP32-C6**: WiFi 6 support, improved CSI capabilities

### Network Requirements

- All ESP32 devices and the Python server must be on the **same local network**
- Router must support **802.11n/g** on **2.4GHz band**
- Static IP assignment recommended for ESP32 devices
- UDP port 5005 must be open and accessible

---

## Installation

### Prerequisites

- Python 3.10 or higher
- pip package manager
- Git (for cloning)

### Step 1: Clone the Repository

```bash
git clone https://github.com/yourusername/csi-tracking-system.git
cd csi-tracking-system
```

### Step 2: Create Virtual Environment (Recommended)

```bash
python -m venv venv

# Linux/macOS
source venv/bin/activate

# Windows
venv\Scripts\activate
```

### Step 3: Install Dependencies

```bash
pip install -r requirements.txt
```

### Step 4: Verify Installation

```bash
python -c "import numpy, scipy, cv2, ultralytics, filterpy; print('All dependencies OK')"
```

### Platform-Specific Notes

#### Ubuntu/Debian

```bash
sudo apt-get update
sudo apt-get install -y python3-dev python3-pip libopencv-dev
```

#### macOS

```bash
brew install opencv
```

#### Windows

Pre-built wheels are available for all packages. Just run `pip install -r requirements.txt`.

---

## Firmware Setup

### Option A: Arduino IDE

1. **Install Arduino IDE** (2.0+ recommended) from [arduino.cc](https://www.arduino.cc/en/software)

2. **Add ESP32 Board Support:**
   - Open `File > Preferences`
   - Add to Additional Board Manager URLs:
     ```
     https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
     ```
   - Open `Tools > Board > Board Manager`
   - Search for "ESP32" and install "ESP32 by Espressif Systems"

3. **Select Board:**
   - `Tools > Board > esp32 > ESP32 Dev Module`
   - Set Upload Speed: `921600`
   - Set CPU Frequency: `240MHz (WiFi/BT)`
   - Set Flash Size: `4MB (32Mb)`
   - Set Partition Scheme: `Default 4MB with spiffs`

4. **Configure WiFi Credentials:**
   - Open `firmware/esp32_csi_tx/config.h`
   - Update the following:
     ```cpp
     #define WIFI_SSID_DEF "YOUR_WIFI_SSID"
     #define WIFI_PASSWORD_DEF "YOUR_WIFI_PASSWORD"
     #define UDP_TARGET_IP_DEF "192.168.1.200"  // Your PC's IP
     ```

5. **Compile and Upload:**
   - Open `firmware/esp32_csi_tx/esp32_csi_tx.ino`
   - Click the Upload button (right arrow)
   - Open Serial Monitor at 921600 baud to verify

### Option B: PlatformIO (Recommended)

1. **Install PlatformIO:**
   ```bash
   pip install platformio
   ```

2. **Create Project Structure:**
   ```
   firmware/esp32_csi_tx/
   ├── platformio.ini
   ├── src/
   │   └── main.cpp        (rename .ino to .cpp)
   └── include/
       └── config.h
   ```

3. **Create `platformio.ini`:**
   ```ini
   [platformio]
   src_dir = src
   
   [env:esp32dev]
   platform = espressif32
   board = esp32dev
   framework = arduino
   monitor_speed = 921600
   monitor_filters = esp32_exception_decoder
   build_flags = 
       -DWIFI_SSID_DEF='"YOUR_WIFI_SSID"'
       -DWIFI_PASSWORD_DEF='"YOUR_WIFI_PASSWORD"'
       -DUDP_TARGET_IP_DEF='"192.168.1.200"'
   upload_speed = 921600
   ```

4. **Build and Upload:**
   ```bash
   cd firmware/esp32_csi_tx
   pio run --target upload
   pio device monitor --baud 921600
   ```

### Finding Your PC's IP Address

```bash
# Linux/macOS
ip addr show | grep "inet " | head -1

# Windows
ipconfig | findstr "IPv4"
```

### Serial Monitor Output (Expected)

```
==================================
ESP32 CSI Transmitter
Version: 1.0.0
==================================
Connecting to YOUR_WIFI_SSID
WiFi Connected!
IP: 192.168.1.100
Gateway: 192.168.1.1

CSI streaming started!
Sample Rate: 100 Hz
Target: 192.168.1.200:5005
==================================
[OK] Packets: 495 | Rate: 99.0 pkt/s
[OK] Packets: 995 | Rate: 99.5 pkt/s
```

---

## Usage

### Single ESP32 Mode (Basic Tracking)

For single-device CSI-based motion detection and basic tracking:

```bash
python main.py
```

This mode:
- Listens for UDP packets from a single ESP32
- Processes CSI data for motion detection
- Displays real-time CSI heatmaps
- Shows activity classification

### Multi-ESP32 Mode (Triangulation)

For multi-device localization with triangulation:

```bash
python main.py --useMultiESP --esp-ips 192.168.1.100 192.168.1.101 192.168.1.102
```

This mode:
- Connects to multiple ESP32 devices
- Collects CSI from each device simultaneously
- Performs time-synchronized analysis
- Estimates position using triangulation
- Shows multi-device heatmaps and position estimates

### With Computer Vision Fusion

```bash
python main.py --useCV --camera-id 0
```

This mode:
- Enables webcam-based person detection (YOLO)
- Fuses CSI motion data with visual detection
- Provides Kalman-filtered tracking
- Shows combined visualization overlay

### Full Multi-ESP + CV Mode

```bash
python main.py \
    --useMultiESP \
    --esp-ips 192.168.1.100 192.168.1.101 192.168.1.102 \
    --useCV \
    --camera-id 0 \
    --config config.yaml
```

### TF-Luna LiDAR Reconstruction (Max Resolution)

Capture TF-Luna over UART and reconstruct a dense 3D cloud (one point per valid sample):

```bash
python tfluna_reconstruct.py \
  --port /dev/ttyUSB0 \
  --baudrate 115200 \
  --seconds 8 \
  --azimuth-min -90 \
  --azimuth-max 90 \
  --elevation 0 \
  --out tfluna_cloud.ply \
  --csv tfluna_capture.csv
```

Notes:
- `tfluna_capture.csv` stores raw + angle-tagged scan samples.
- `tfluna_cloud.ply` stores reconstructed XYZ points at max capture resolution.
- Use real servo/encoder angles for best geometric accuracy.

### Command-Line Arguments

| Argument | Type | Default | Description |
|----------|------|---------|-------------|
| `--useMultiESP` | flag | False | Enable multi-ESP32 mode |
| `--esp-ips` | list | [] | IP addresses of ESP32 devices |
| `--useCV` | flag | False | Enable computer vision fusion |
| `--camera-id` | int | 0 | Camera device index |
| `--config` | str | config.yaml | Path to configuration file |
| `--udp-port` | int | 5005 | UDP port for CSI data |
| `--visualize` | flag | True | Enable real-time visualization |
| `--save-data` | str | None | Path to save CSI data (CSV) |
| `--no-gui` | flag | False | Run without GUI (headless) |

---

## Configuration

### YAML Configuration File (`config.yaml`)

```yaml
# ============================================================
# CSI Tracking System Configuration
# ============================================================

# -----------------------------------------------------------
# System Settings
# -----------------------------------------------------------
system:
  name: "CSI Tracking System"
  version: "1.0.0"
  log_level: "INFO"  # DEBUG, INFO, WARNING, ERROR
  data_directory: "./data"

# -----------------------------------------------------------
# Network Configuration
# -----------------------------------------------------------
network:
  udp_port: 5005
  udp_buffer_size: 65536
  packet_timeout_ms: 100
  max_packet_size: 2048
  
# -----------------------------------------------------------
# ESP32 Device Configuration
# -----------------------------------------------------------
esp32:
  devices:
    - id: "esp32_a"
      ip: "192.168.1.100"
      position: [0.0, 0.0]  # Room coordinates (meters)
      sample_rate_hz: 100
      
    - id: "esp32_b"
      ip: "192.168.1.101"
      position: [5.0, 0.0]
      sample_rate_hz: 100
      
    - id: "esp32_c"
      ip: "192.168.1.102"
      position: [2.5, 5.0]
      sample_rate_hz: 100
      
  num_subcarriers: 64
  csi_data_type: "int8"

# -----------------------------------------------------------
# Signal Processing Configuration
# -----------------------------------------------------------
signal_processing:
  # Noise filtering
  hampel_window: 7
  hampel_threshold: 3.0
  
  # Phase sanitization
  phase_sanitization: true
  phase_unwrap: true
  
  # Subcarrier selection
  subcarrier_selection:
    method: "variance"  # variance, correlation, all
    top_k: 20           # Number of subcarriers to select
    
  # Moving average smoothing
  smoothing_window: 5
  
  # Activity detection
  activity_threshold: 0.15
  min_activity_duration_ms: 500

# -----------------------------------------------------------
# Computer Vision Configuration
# -----------------------------------------------------------
computer_vision:
  enabled: false
  camera_id: 0
  camera_width: 1280
  camera_height: 720
  camera_fps: 30
  
  # YOLO model
  yolo_model: "yolov8n.pt"
  confidence_threshold: 0.5
  
  # Tracking
  tracker_type: "kalman"
  max_track_age: 30
  
  # Kalman filter parameters
  kalman:
    process_noise: 0.01
    measurement_noise: 0.1
    initial_covariance: 1.0

# -----------------------------------------------------------
# Fusion Configuration
# -----------------------------------------------------------
fusion:
  method: "weighted"  # weighted, kalman, bayesian
  csi_weight: 0.4
  cv_weight: 0.6
  
# -----------------------------------------------------------
# Visualization Configuration
# -----------------------------------------------------------
visualization:
  enabled: true
  csi_heatmap:
    colormap: "viridis"
    update_interval_ms: 100
    
  trajectory_plot:
    history_length: 200
    show_confidence: true
    
  signal_plot:
    num_subcarriers_display: 10
    time_window_seconds: 5.0
    
  layout: "grid"  # grid, tabs, separate

# -----------------------------------------------------------
# Data Recording
# -----------------------------------------------------------
data_recording:
  enabled: false
  output_directory: "./recordings"
  filename_template: "csi_capture_{timestamp}.csv"
  max_file_size_mb: 100
  auto_rotate: true
```

---

## Module Descriptions

### `src/`

| Module | Description |
|--------|-------------|
| `__init__.py` | Package initialization |
| `udp_receiver.py` | UDP binary frame receiver and parser |
| `csi_parser.py` | CSI data extraction from binary frames |
| `signal_processor.py` | Noise filtering, phase unwrap, subcarrier selection |
| `motion_detector.py` | Hampel filter, variance analysis, activity classification |
| `triangulation.py` | Multi-ESP position estimation using triangulation |
| `cv_tracker.py` | YOLO-based person detection and Kalman tracking |
| `fusion_engine.py` | Sensor fusion between CSI and computer vision |
| `visualizer.py` | Real-time matplotlib/OpenCV visualization |
| `config_loader.py` | YAML configuration file loader and validator |
| `main.py` | Application entry point |

### Data Flow Pipeline

```
UDP Packet (binary)
    |
    v
[udp_receiver.py] --> Parse binary frame
    |
    v
[csi_parser.py] --> Extract I/Q pairs, RSSI, timestamp
    |
    v
[signal_processor.py] --> Filter, unwrap, select subcarriers
    |
    +-------> [motion_detector.py] --> Activity classification
    |                                   (static, walking, running)
    |
    +-------> [triangulation.py] --> Position estimate (multi-ESP)
    |
    +-------> [cv_tracker.py] --> Visual detection & tracking
    |
    v
[fusion_engine.py] --> Combined position estimate
    |
    v
[visualizer.py] --> Real-time display
```

---

## API Reference

### UDP Binary Frame Protocol

The ESP32 sends CSI data over UDP in a structured binary format for efficient parsing.

**Frame Structure:**

| Field | Size | Type | Description |
|-------|------|------|-------------|
| Magic | 4 bytes | uint32 | Frame sync: `0xC5110001` |
| Timestamp | 8 bytes | double | Seconds since ESP32 boot |
| RSSI | 4 bytes | int32 | Received Signal Strength Indicator (dBm) |
| CSI Length | 4 bytes | uint32 | Number of bytes in CSI payload |
| CSI Data | N bytes | int8[] | I/Q pairs (interleaved) |

**CSI Data Format:**

The CSI payload contains interleaved signed 8-bit I (in-phase) and Q (quadrature) samples:

```
[I0, Q0, I1, Q1, I2, Q2, ..., IN, QN]
```

Each subcarrier has one I/Q pair. The ESP32 typically provides 64-256 subcarriers depending on configuration.

**Parsing Example (Python):**

```python
import struct
import numpy as np

def parse_csi_frame(data: bytes) -> dict:
    magic = struct.unpack('<I', data[0:4])[0]
    if magic != 0xC5110001:
        raise ValueError(f"Invalid magic: 0x{magic:08X}")
    
    timestamp = struct.unpack('<d', data[4:12])[0]
    rssi = struct.unpack('<i', data[12:16])[0]
    csi_len = struct.unpack('<I', data[16:20])[0]
    
    csi_raw = np.frombuffer(data[20:20+csi_len], dtype=np.int8)
    csi_complex = csi_raw[0::2] + 1j * csi_raw[1::2]  # I/Q pairs
    
    return {
        'timestamp': timestamp,
        'rssi': rssi,
        'csi': csi_complex,
        'num_subcarriers': len(csi_complex)
    }
```

---

## Troubleshooting

### Common Issues

#### ESP32 won't connect to WiFi

- **Check credentials**: Verify SSID and password in `config.h`
- **2.4GHz band**: ESP32 only supports 2.4GHz WiFi
- **Signal strength**: Ensure ESP32 is within range of router
- **Router compatibility**: Some enterprise routers block ESP32 connections

**Solution:**
```cpp
// Add this debug output to check connection state
Serial.print("WiFi Status: ");
Serial.println(WiFi.status());
// 0 = IDLE, 1 = NO_SSID_AVAIL, 3 = CONNECTED, 4 = CONNECT_FAILED, 6 = DISCONNECTED
```

#### No UDP packets received

- **Check firewall**: Ensure UDP port 5005 is open
- **Verify IP**: Confirm Python server IP matches ESP32 target
- **Same network**: Both devices must be on the same subnet

**Solution:**
```bash
# Test UDP connectivity
python -c "
import socket
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(('0.0.0.0', 5005))
print('Listening on 0.0.0.0:5005...')
data, addr = s.recvfrom(2048)
print(f'Received {len(data)} bytes from {addr}')
"
```

#### CSI data is all zeros

- **Router must send ACK frames**: CSI is generated on received packets
- **Enable promiscuous mode**: Some routers don't broadcast enough
- **Increase ping rate**: Lower `CSI_PACKET_RATE_MS` to send more packets

**Solution:**
```cpp
// Increase ping frequency to generate more traffic
#define CSI_PACKET_RATE_DEF 5  // 200 Hz

// Or send to broadcast address
udp.beginPacket(IPAddress(192, 168, 1, 255), UDP_TARGET_PORT_DEF);
```

#### High packet loss

- **Reduce packet size**: CSI payload can be large; decrease if needed
- **Check WiFi quality**: Poor signal causes retransmissions
- **Reduce sample rate**: Lower to 50 Hz if 100 Hz causes issues

**Solution:**
```cpp
// Reduce sample rate
#define CSI_SAMPLE_RATE_HZ 50
#define CSI_PACKET_RATE_DEF 20
```

#### Python import errors

```bash
# Reinstall in clean environment
pip install --upgrade pip
pip install -r requirements.txt --force-reinstall

# Check package versions
pip list | grep -E "numpy|scipy|opencv|ultralytics|filterpy"
```

#### Camera not detected

```bash
# List available cameras
python -c "
import cv2
for i in range(10):
    cap = cv2.VideoCapture(i)
    if cap.isOpened():
        print(f'Camera {i}: {cap.get(cv2.CAP_PROP_FRAME_WIDTH)}x{cap.get(cv2.CAP_PROP_FRAME_HEIGHT)}')
        cap.release()
"
```

### Debug Mode

Enable verbose logging:

```yaml
# config.yaml
system:
  log_level: "DEBUG"
```

```bash
# Run with debug output
python main.py --config config.yaml 2>&1 | tee debug.log
```

### Performance Optimization

| Bottleneck | Solution | Expected Improvement |
|------------|----------|---------------------|
| UDP packet drops | Increase `udp_buffer_size` to 256KB | 50% fewer drops |
| Slow visualization | Reduce `update_interval_ms` to 250 | 40% CPU reduction |
| YOLO inference lag | Use `yolov8n.pt` (nano model) | 5x faster inference |
| High memory usage | Reduce `history_length` to 100 | 50% memory reduction |

---

## Research & Citations

This project is built upon foundational research in WiFi-based sensing:

### Key Papers

1. **Xie et al. (2015)** - "Precise Power Delay Profiling with Commodity WiFi"
   - Introduced CSI-based motion detection using commercial hardware
   - *ACM MobiCom 2015*

2. **Pu et al. (2013)** - "Whole-Home Gesture Recognition Using Wireless Signals"
   - Demonstrated whole-home gesture recognition using WiFi CSI
   - *ACM MobiCom 2013*

3. **Wang et al. (2014)** - "E-eyes: Device-free Location-oriented Activity Identification"
   - Activity recognition using fine-grained WiFi signatures
   - *ACM MobiSys 2014*

4. **Youssef et al. (2007)** - "Challenges: Device-free Passive Localization for Wireless Environments"
   - Foundation of device-free localization using WiFi signals
   - *ACM MobiCom 2007*

5. **Kotaru et al. (2015)** - "SpotFi: Decimeter Level Localization Using WiFi"
   - High-accuracy indoor localization using CSI phase information
   - *ACM SIGCOMM 2015*

6. **Redmon et al. (2016)** - "You Only Look Once: Unified, Real-Time Object Detection"
   - YOLO architecture for real-time object detection
   - *CVPR 2016*

### ESP32 CSI References

- [Espressif ESP-IDF WiFi CSI Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/wifi.html#wi-fi-channel-state-information)
- [ESP32 CSI Open Source Project](https://github.com/StevenMHernandez/ESP32-CSI-Tool)
- [ESP32 Arduino Core WiFi Examples](https://github.com/espressif/arduino-esp32/tree/master/libraries/WiFi)

### Open Source Dependencies

| Package | License | Purpose |
|---------|---------|---------|
| NumPy | BSD-3 | Numerical computing |
| SciPy | BSD-3 | Signal processing |
| OpenCV | Apache-2.0 | Computer vision |
| Ultralytics (YOLO) | AGPL-3.0 | Object detection |
| FilterPy | MIT | Kalman filtering |
| Matplotlib | PSF-based | Visualization |

---

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE) file for details.

---

## Contributing

Contributions are welcome! Please see [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

---

## Acknowledgments

- Steven M. Hernandez for the ESP32-CSI-Tool project
- The ESP32 Arduino Core team at Espressif
- Ultralytics for the YOLO implementation
- The open-source signal processing community
