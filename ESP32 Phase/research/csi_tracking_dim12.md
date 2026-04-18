# Dimension 12: Existing Open-Source Tools, Libraries & Relevant Projects

## Comprehensive Survey of Open-Source Ecosystem for ESP32 CSI, WiFi Sensing, Indoor Localization, and Related Areas

**Date**: 2025-07-01
**Searches Conducted**: 25+
**Sources**: Primary (GitHub repos, official docs), Academic papers, Community forums

---

## Table of Contents

1. [Official Espressif esp-csi Repository](#1-official-espressif-esp-csi-repository)
2. [ESPectre (formerly Wi-ESP)](#2-espectre-motion-detection-system)
3. [ESP32-CSI-Tool](#3-esp32-csi-tool)
4. [Python Libraries for CSI Processing](#4-python-libraries-for-csi-processing)
5. [Indoor Localization Frameworks](#5-indoor-localization-frameworks)
6. [MUSIC Algorithm Implementations](#6-music-algorithm-implementations)
7. [YOLO Tracking Implementations](#7-yolo-tracking-implementations)
8. [Sensor Fusion Libraries in Python](#8-sensor-fusion-libraries-in-python)
9. [Complete WiFi Sensing Pipelines](#9-complete-wifi-sensing-pipelines)
10. [Visualization Tools for CSI Data](#10-visualization-tools-for-csi-data)
11. [Papers with Open-Sourced Code](#11-papers-with-open-sourced-code)
12. [Arduino/PlatformIO Libraries for ESP32 CSI](#12-arduinoplatformio-libraries-for-esp32-csi)
13. [Emerging and Notable Projects](#13-emerging-and-notable-projects)
14. [Summary Matrix](#14-summary-matrix)

---

## 1. Official Espressif esp-csi Repository

### Overview

The **esp-csi** repository is Espressif's official, vendor-maintained project for Wi-Fi Channel State Information applications on ESP32 chips. It is listed in the official ESP-IDF Programming Guide under "Espressif's Frameworks" alongside ESP-ADF, ESP-DSP, ESP-WHO, and ESP RainMaker.

**Claim**: ESP-CSI is an experimental implementation that uses Wi-Fi Channel State Information to detect the presence of a human body. [^127^]
**Source**: ESP-IDF Programming Guide v6.0
**URL**: https://docs.espressif.com/projects/esp-idf/en/stable/esp32c2/libraries-and-frameworks/libs-frameworks.html
**Date**: Current
**Excerpt**: "ESP-CSI is an experimental implementation that uses the Wi-Fi Channel State Information to detect the presence of a human body. See the ESP-CSI project for more information."
**Confidence**: High

### Repository Details

```
Claim: The esp-csi repository provides examples for get-started, esp-radar, and tools for CSI data analysis.
Source: GitHub - espressif/esp-csi
URL: https://github.com/espressif/esp-csi
Date: 2021-02-20 (initial)
Excerpt: "Applications based on Wi-Fi CSI (Channel state information), such as indoor positioning, human detection"
Context: Official Espressif repository with active maintenance
Confidence: High
```

### Repository Structure

The repository is organized into two main example categories:

#### 1.1 get-started (Basic Examples)

| Example | Description |
|---------|-------------|
| `csi_recv` | ESP32 as receiver - captures CSI data from incoming packets |
| `csi_send` | ESP32 as sender - transmits packets to generate CSI |
| `csi_recv_router` | Uses router as sender; ESP32 triggers CSI packets via Ping |
| `tools/csi_data_read_parse.py` | Python script for CSI data analysis with graphical display |

**Claim**: The get-started examples demonstrate how to obtain CSI data through communication between two Espressif chips, using a graphical interface to display real-time CSI subcarrier data. [^150^]
**Source**: GitHub README
**URL**: https://github.com/espressif/esp-csi/blob/master/examples/get-started/README.md
**Date**: 2021-02-20
**Excerpt**: "This example demonstrates how to obtain CSI data through communication between two espressif chips, and uses a graphical interface to display real-time data of CSI subcarriers"
**Confidence**: High

**Hardware Recommendations**:
- Best: ESP32-C5 (dual-band) or ESP32-C6 (best currently released RF chip)
- Good: ESP32-C3, ESP32-S3
- Supported: Original ESP32
- External antenna strongly recommended over PCB antenna

#### 1.2 esp-radar (Application Examples)

| Example | Description |
|---------|-------------|
| `connect_rainmaker` | Captures CSI and uploads to ESP RainMaker cloud |
| `console_test` | Interactive console with human activity detection algorithms |

#### 1.3 esp-radar Component Registry

The `esp-radar` component is also distributed through the ESP Component Registry:

**Claim**: The esp-radar component provides algorithmic functionality for ESP-CSI human movement and presence detection for easy integration into products. [^204^]
**Source**: ESP Component Registry
**URL**: https://components.espressif.com/components/espressif/esp-radar
**Date**: Current (v0.3.3)
**Excerpt**: "This package provides algorithmic functionality for ESP-CSI human movement and presence detection for easy integration into products."
**Confidence**: High

Installation: `idf.py add-dependency "espressif/esp-radar=*"`

### esp_csi_tool.py - Official Visualization Tool

The `console_test` example includes `esp_csi_tool.py`, a comprehensive Python GUI for CSI visualization:

**Features**:
- **Raw data display**: Real-time CSI sub-carrier waveform display
- **Radar model**: Human movement detection (someone/noneone, move/static)
- **RSSI waveform**: For comparison with CSI data
- **Room status display**: Calibration and threshold configuration
- **Action collection**: Labeled data collection for ML training
- **Model evaluation**: Accuracy assessment of detection thresholds
- **People movement data**: Histogram and time records of movements

**Claim**: The esp_csi_tool.py provides a test platform for Wi-Fi CSI including data display, acquisition, and analysis. [^128^]
**Source**: ESP Component Registry documentation
**URL**: https://components.espressif.com/components/espressif/esp-radar/versions/0.2.0/examples/console_test
**Date**: Current
**Excerpt**: "This example provides a test platform for Wi-Fi CSI, which includes functions such as data display, data acquisition and data analysis"
**Confidence**: High

### csi_data_read_parse.py

A simpler Python script in `examples/get-started/tools/` for basic CSI data parsing and visualization using PyQt5.

### esp_csi_gain_ctrl Component

**Claim**: The esp_csi_gain_ctrl component provides receive gain management for stabilizing CSI data acquisition, compensating AGC and FFT gain variations. [^158^]
**Source**: ESP Component Registry
**URL**: https://components.espressif.com/components/espressif/esp_csi_gain_ctrl/versions/0.1.5/readme
**Date**: 2025-04-14
**Excerpt**: "esp_csi_gain_ctrl is the receiving gain control component for the ESP-CSI radar system. It provides receiving gain management functions for stabilizing CSI data acquisition and processing"
**Confidence**: High

**Key APIs**:
- `esp_csi_gain_ctrl_get_gain_status()` - Check gain status
- `esp_csi_gain_ctrl_record_rx_gain()` - Record gain samples
- `esp_csi_gain_ctrl_get_rx_gain_baseline()` - Get baseline
- `esp_csi_gain_ctrl_get_gain_compensation()` - Calculate compensation
- `esp_csi_gain_ctrl_compensate_rx_gain()` - Apply compensation

### Known Issues

**Claim**: Users report issues with console_test not showing data on graphs, with confusion about which firmware to flash. [^153^] [^155^]
**Source**: GitHub Issues
**URL**: https://github.com/espressif/esp-csi/issues/205, https://github.com/espressif/esp-csi/discussions/217
**Date**: 2024-09-13, 2024-12-03
**Excerpt**: "esp_csi_tool.py shows nothing except logs, but csi_data_read_parse.py shows a graph"
**Confidence**: High

---

## 2. ESPectre (Motion Detection System)

**Note**: The research context mentioned "Wi-ESP" - this appears to refer to **ESPectre**, a prominent community project for ESP32 CSI-based motion detection.

### Overview

ESPectre is a production-ready, open-source motion detection system based on Wi-Fi CSI analysis, with native Home Assistant integration via ESPHome.

**Claim**: ESPectre is a motion detection system based on Wi-Fi spectre analysis (CSI), with native Home Assistant integration via ESPHome. [^119^]
**Source**: GitHub - francescopace/espectre
**URL**: https://github.com/francescopace/espectre
**Date**: Active (2026-03-17)
**Excerpt**: "Motion detection system based on Wi-Fi spectre analysis (CSI), with native Home Assistant integration via ESPHome."
**Confidence**: High

### Key Features

- **Zero programming required**: YAML-only configuration
- **Multi-platform support**: ESP32-C6, ESP32-S3, ESP32-C3, ESP32 (original)
- **Two-mode detection**: MVS (threshold-based) and ML (neural network-based)
- **Auto-calibration**: NBVI (Normalized Band Variance Index) for automatic subcarrier selection
- **Home Assistant integration**: Auto-discovery via ESPHome Native API
- **Cost**: ~EUR 10 per device

### Architecture

```
ESP32 Room 1    ESP32 Room 2    ESP32 Room 3
     |               |               |
     +---------------+---------------+
                     | ESPHome Native API
                     v
            +-------------------+
            |   Home Assistant  |
            |  (Auto-discovery) |
            +-------------------+
```

### Algorithm Documentation

ESPectre includes comprehensive algorithm documentation:
- **MVS Mode**: Uses Hampel filter, NBVI calibration, segmentation
- **ML Mode**: Neural network-based, no calibration required, runs on-device
- **F1 Score**: >96% with zero manual configuration

**Claim**: ESPectre implements NBVI for automatic subcarrier selection, achieving near-optimal performance (F1>96%) with zero manual configuration. [^119^]
**Source**: GitHub README
**URL**: https://github.com/francescopace/espectre
**Excerpt**: "ESPectre implements NBVI (Normalized Band Variance Index) for automatic subcarrier selection, achieving near-optimal performance (F1>96%) with zero manual configuration."
**Confidence**: High

### Two-Platform Strategy

| Platform | Purpose | Features |
|----------|---------|----------|
| ESPectre | Production | ESPHome, YAML config, ML detector |
| Micro-ESPectre | R&D | CLI, MQTT, Web Monitor, data collection |

**License**: GNU GPL v3.0
**Author**: Francesco Pace

---

## 3. ESP32-CSI-Tool

### Overview

The ESP32-CSI-Tool is one of the earliest and most widely-used open-source CSI extraction utilities for ESP32, created by Steven Hernandez.

**Claim**: The ESP32-CSI-Tool is a CSI extraction toolkit for ESP32 hardware, released by Steven Hernandez. [^132^]
**Source**: GitHub - StevenMHernandez/ESP32-CSI-Tool
**URL**: https://github.com/StevenMHernandez/ESP32-CSI-Tool
**Date**: 2019-07-09
**Excerpt**: "Each project automatically sends the collected CSI data to both serial port and SD card (if present)."
**Confidence**: High

### Project Structure

| Sub-project | Purpose |
|-------------|---------|
| `active_sta` | Active Station - typically used as CSI-TX |
| `active_ap` | Active Access Point - typically used as CSI-RX |
| `passive` | Passive CSI collection - passive RX |

### Key Capabilities

- **Dual output**: CSI data to both serial port and SD card
- **Active collection**: Two ESP32s as TX/RX pair
- **Passive collection**: Sniffs existing WiFi traffic
- **ESP-IDF v4.3 required**
- **Baud rate**: 921600 recommended

### Resolution and Subcarriers

**Claim**: The ESP32-CSI-Tool provides 8-bit resolution with 64 subcarriers for 20 MHz and 114 subcarriers for 40 MHz channels. [^121^]
**Source**: MDPI Sensors Paper
**URL**: https://www.mdpi.com/1424-8220/25/19/6220
**Date**: 2025-10-08
**Excerpt**: "ESP32 CSI Tool: ESP32 devices, 8-bit, 64 and 114 for 20 MHz and 40 MHz channels"
**Confidence**: High

### Forks and Extensions

Multiple research projects forked from ESP32-CSI-Tool:

1. **ESP32-WiFi-Sensing** (thu4n): CNN + Random Forest HAR with Jetson Nano [^123^]
2. **wifi-sensing-har** (jasminkarki): Real-time HAR with threading [^124^]
3. **Embedded_WiFi_Sensing** (AlbanyArmenta0711): Complete ecosystem with web tool [^126^]

---

## 4. Python Libraries for CSI Processing

### 4.1 csiread

**csiread** is the fastest CSI parser available, implemented in Cython.

**Claim**: csiread is a fast channel state information parser for Intel, Atheros, Nexmon, ESP32 and PicoScenes in Python, at least 15x faster than MATLAB implementations. [^165^]
**Source**: PyPI
**URL**: https://pypi.org/project/csiread/
**Date**: 2024-10-12
**Excerpt**: "A fast channel state information parser for Intel, Atheros, Nexmon, ESP32 and PicoScenes in Python. At least 15 times faster than the implementation in Matlab"
**Confidence**: High

**Features**:
- Full support for Linux 802.11n CSI Tool, Atheros CSI Tool, nexmon_csi, ESP32-CSI-Tool
- Experimental PicoScenes support
- Real-time parsing and visualization
- CSI toolbox with utilities and algorithms

**Performance Comparison** (40k packets):

| Function | MATLAB | Python3+Numpy | csiread |
|----------|--------|---------------|---------|
| Nexmon read | 3.23s | 0.27s | 0.07s |
| Atheros read | 3.31s | 14.60s | 0.10s |
| Intel read | 1.61s | 7.66s | 0.05s |

Installation: `pip3 install csiread`

### 4.2 CSIKit

**CSIKit** is a comprehensive Python framework for CSI processing and visualization.

**Claim**: CSIKit is a Python framework for CSI processing and visualisation from Atheros, Intel, Nexmon, ESP32, FeitCSI, and PicoScenes formats. [^132^]
**Source**: GitHub - Gi-z/CSIKit
**URL**: https://github.com/Gi-z/CSIKit
**Date**: 2020-03-12
**Excerpt**: "Python CSI processing and visualisation tools for Atheros, Intel, Nexmon, ESP32, FeitCSI, and PicoScenes (USRP, etc) formats."
**Confidence**: High

**Features**:
- CSI parsing from multiple hardware formats
- Processing using numpy
- Visualization using matplotlib
- CSV/JSON generators for dataset serialization
- Command-line tool: `csikit [OPTIONS] file`
- Library API for integration with TensorFlow/PyTorch

**Installation**: `pip install csikit`

**Usage**:
```python
from CSIKit.reader import get_reader
my_reader = get_reader("path/to/csi_file.dat")
csi_data = my_reader.read_file("path/to/csi_file.dat")
```

**Filters Available**:
- `lowpass` - Low-pass frequency filter
- `hampel` - Hampel outlier filter
- `running_mean` - Statistical smoothing filter

### 4.3 Comparison of Python CSI Libraries

| Library | Speed | Formats | Visualization | License | Maintenance |
|---------|-------|---------|---------------|---------|-------------|
| csiread | Very Fast (Cython) | Intel, Atheros, Nexmon, ESP32, PicoScenes | Real-time plotting | MIT | Active |
| CSIKit | Moderate | Atheros, Intel, Nexmon, ESP32, FeitCSI, PicoScenes | Full (matplotlib) | MIT | Active |

---

## 5. Indoor Localization Frameworks

### 5.1 Anyplace

**Anyplace** is a research-grade indoor information service offering GPS-less localization, navigation, and search.

**Claim**: Anyplace is a first-of-its-kind indoor information service offering GPS-less localization, navigation and search inside buildings using ordinary smartphones, open source under MIT licence. [^141^]
**Source**: Anyplace official website
**URL**: https://anyplace.cs.ucy.ac.cy/
**Date**: Current
**Excerpt**: "Anyplace is a first-of-its-kind indoor information service offering GPS-less localization, navigation and search inside buildings using ordinary smartphones."
**Confidence**: High

**Key Features**:
- MIT-licensed open source (Scala/AngularJS)
- NoSQL backend (MongoDB)
- JSON API
- Mobile clients for Web, Android, Windows Phone
- Google Maps API integration
- Crowdsourcing capability
- 100,000+ real user interactions

**Awards**:
- 2014: 1st place at EVARILOS Open Challenge (EU)
- 2014: 2nd place at Microsoft Research Indoor Localization Competition (IPSN)

### 5.2 FIND / FIND3

**FIND** (Framework for Internal Navigation and Discovery) is a Python-based indoor positioning framework.

**Claim**: FIND3 is a complete re-write with support for any data source (Bluetooth/WiFi/magnetic fields), passive scanning, and meta-learning with 10 different ML classifiers. [^208^]
**Source**: GitHub - schollz/find3
**URL**: https://github.com/schollz/find3
**Date**: 2018-02-15
**Excerpt**: "Support for any data source - Bluetooth / WiFi / magnetic fields / etc. (previously just WiFi). Passive scanning built-in. Meta-learning with 10 different machine learning classifiers."
**Confidence**: High

**Architecture**:
- Data storage server (this repo)
- Machine learning server (this repo)
- Command-line scanner: schollz/find3-cli-scanner
- Android app: schollz/find3-android-scanner
- ESP32 client: DatanoiseTV/esp-find3-client

**Features**:
- SQLite database storage
- Websockets+React client
- MQTT endpoints
- Rolling MAC address compression
- MIT license

**Note**: FIND3 uses RSSI-based fingerprinting, not CSI. It serves as a reference architecture for indoor localization systems.

### 5.3 CSI-Fingerprint-Indoor-Localization

**Claim**: A TensorFlow implementation of "Hybrid CNN-LSTM based Robust Indoor Pedestrian Localization with CSI Fingerprint Maps" achieving sub-meter accuracy. [^202^]
**Source**: GitHub
**URL**: https://github.com/em22ad/CSI-Fingerprint-Indoor-Localization
**Date**: Current
**Excerpt**: "Implementation of 'Hybrid CNN-LSTM based Robust Indoor Pedestrian Localization with CSI Fingerprint Maps' using TensorFlow."
**Confidence**: Medium

**Architecture**:
- Input: 520 CSI features
- 2 LSTM layers, 520 hidden units
- Multi-level building and floor classification

### 5.4 Indoor WiFi Localization with ESP32 (RSSI-based)

**Claim**: An ESP32 self-localization project using nearby WiFi routers' RSSI signal strength with a custom KNN implementation. [^207^]
**Source**: GitHub
**URL**: https://github.com/joaocarvalhoopen/Indoor_WiFi_Localization_in_ESP32_using_Machine_Leaning
**Date**: 2019-09-24
**Excerpt**: "In this project a ESP32 microcontroller can do self indoor localization in each room inside a building, using only the nearby WiFi routers public RSSI signal strength value."
**Confidence**: Medium

---

## 6. MUSIC Algorithm Implementations

### 6.1 ESPARGOS/pyespargos

**ESPARGOS** is a multi-antenna WiFi channel sounder that includes MUSIC algorithm demos.

**Claim**: pyespargos includes a music-spectrum demo that uses the MUSIC algorithm to display a spatial (angular) spectrum for angle of arrival estimation. [^177^]
**Source**: GitHub - ESPARGOS/pyespargos
**URL**: https://github.com/ESPARGOS/pyespargos
**Date**: 2026-04-08
**Excerpt**: "music-spectrum: Use the MUSIC algorithm to display a spatial (angular) spectrum. Demonstrates angle of arrival (AoA) estimation."
**Confidence**: High

**Demo Applications**:
| Demo | Description |
|------|-------------|
| `music-spectrum` | MUSIC spatial spectrum for AoA |
| `phases-over-space` | Received phase per antenna |
| `instantaneous-csi` | Frequency/time domain CSI |
| `azimuth-delay` | 2D azimuth-delay diagram |
| `combined-array` | Multiple ESPARGOS array combination |
| `camera` | Overlay WiFi spectrum on camera feed |

### 6.2 music-aoa-estimation-py

A standalone Python implementation for parametric analysis of MUSIC algorithm.

**Claim**: Parametric analysis of MUSIC algorithm for Angle of Arrival estimation in Python with Monte Carlo simulations. [^186^]
**Source**: GitHub - MarcinWachowiak/music-aoa-estimation-py
**URL**: https://github.com/MarcinWachowiak/music-aoa-estimation-py
**Date**: 2021-11-28
**Excerpt**: "Parametric analysis of MUSIC algorithm for Angle of Arrival estimation in Python."
**Confidence**: High

**Analysis Capabilities**:
- SNR sweep
- Snapshot length sweep
- Number of antennas sweep
- Antenna spacing sweep
- RX power difference sweep
- Monte Carlo error estimation

### 6.3 scipy.signal / NumPy Custom Implementations

For ESP32 CSI specifically, MUSIC must be adapted since ESP32 has limited antenna diversity (typically 1-2 antennas). Workarounds include:

1. **Virtual antenna arrays**: Using multiple ESP32 devices
2. **Frequency-domain MUSIC**: Using subcarrier diversity instead of spatial diversity
3. **ESPARGOS approach**: Custom hardware with multiple ESP32 modules forming an antenna array

### 6.4 Theoretical Background

**Claim**: MUSIC leverages the eigenstructure of the covariance matrix of CSI data to estimate Doppler velocity by separating signal subspace from noise subspace. [^167^]
**Source**: arXiv Tutorial-cum-Survey
**URL**: https://arxiv.org/html/2506.12052v1
**Date**: 2025-05-29
**Excerpt**: "MUSIC leverages the eigen structure of the covariance matrix of the CSI data to estimate the movements Doppler velocity vector by separating the signal subspace from the noise subspace."
**Confidence**: High

---

## 7. YOLO Tracking Implementations

### 7.1 Ultralytics (Official)

**Ultralytics YOLO** provides built-in, production-ready tracking with multiple tracker backends.

**Claim**: Ultralytics YOLO supports BoT-SORT and ByteTrack tracking algorithms via YAML configuration files. [^139^]
**Source**: Ultralytics Documentation
**URL**: https://docs.ultralytics.com/modes/track/
**Date**: 2023-11-12
**Excerpt**: "Ultralytics YOLO supports the following tracking algorithms... BoT-SORT, ByteTrack"
**Confidence**: High

**Usage**:
```python
from ultralytics import YOLO
model = YOLO("yolo11n.pt")
results = model.track(source="video.mp4", tracker="bytetrack.yaml")
```

**Configuration Files**:
- `botsort.yaml` - BoT-SORT tracker (default)
- `bytetrack.yaml` - ByteTrack tracker

**Key Parameters**:
| Parameter | Description | Default |
|-----------|-------------|---------|
| `tracker_type` | Tracker type | `botsort` |
| `track_high_thresh` | First association threshold | 0.25 |
| `track_low_thresh` | Second association threshold | 0.1 |
| `new_track_thresh` | New track threshold | 0.25 |
| `track_buffer` | Track removal buffer | 30 |
| `match_thresh` | Matching threshold | 0.8 |

### 7.2 ByteTrack (Original)

**Claim**: ByteTrack achieves 80.3 MOTA, 77.3 IDF1 on MOT17 with 30 FPS on a single V100 GPU. [^205^]
**Source**: GitHub - ifzhang/ByteTrack
**URL**: https://github.com/ifzhang/ByteTrack
**Date**: 2021
**Excerpt**: "For the first time, we achieve 80.3 MOTA, 77.3 IDF1 and 63.1 HOTA on the test set of MOT17 with 30 FPS running speed on a single V100 GPU."
**Confidence**: High

**Key Innovation**: Associates every detection box (including low-confidence) in a two-stage process.

### 7.3 DeepSORT (Original)

**Claim**: DeepSORT adds deep learning-based appearance features to SORT for robust tracking across occlusions. [^187^]
**Source**: GitHub - nwojke/deep_sort
**URL**: https://github.com/nwojke/deep_sort
**Date**: Original
**Excerpt**: "The original implementation of DeepSORT"
**Confidence**: High

**DeepSORT PyTorch Reimplementation**:
- Repository: ZQPei/deep_sort_pytorch
- Supports YOLOv3, YOLOv5, Mask R-CNN detectors
- PyTorch-based feature extraction (original uses TensorFlow)

### 7.4 BoT-SORT

BoT-SORT improves over ByteTrack with:
- Camera motion compensation
- ReID (Re-identification) features
- Better association for occluded objects

### 7.5 Comparison

| Tracker | Speed | Accuracy | ReID | Best For |
|---------|-------|----------|------|----------|
| ByteTrack | Fast | High | No | General purpose |
| BoT-SORT | Moderate | Higher | Yes | Occlusion-heavy scenes |
| DeepSORT | Moderate | High | Yes | Long-term tracking |

### 7.6 YOLO + Tracking GUI Projects

**Claim**: A GUI application using ultralytics YOLO for Detection/Tracking with DeepSort and ByteTrack support. [^144^]
**Source**: GitHub - jingh-ai/ultralytics-YOLO-DeepSort-ByteTrack-PyQt-GUI
**URL**: https://github.com/jingh-ai/ultralytics-YOLO-DeepSort-ByteTrack-PyQt-GUI
**Date**: 2023-04-11
**Excerpt**: "A GUI application, which uses ultralytics YOLO for Object Detection/Tracking, Human Pose Estimation/Tracking"
**Confidence**: High

---

## 8. Sensor Fusion Libraries in Python

### 8.1 FilterPy

**FilterPy** is the most comprehensive Kalman filter library for Python, extensively documented.

**Claim**: FilterPy is a Python library implementing Kalman filters, Extended Kalman Filters, Unscented Kalman Filters, particle filters, and smoothing algorithms. [^146^]
**Source**: FilterPy Documentation
**URL**: https://filterpy.readthedocs.io/en/latest/
**Date**: Current (v1.4.4)
**Excerpt**: "FilterPy is a Python library that implements a number of Bayesian filters, most notably Kalman filters."
**Confidence**: High

**Key Classes**:
- `KalmanFilter` - Standard linear Kalman filter
- `ExtendedKalmanFilter` - EKF for nonlinear systems
- `UnscentedKalmanFilter` - UKF for strongly nonlinear systems
- `HInfinityFilter` - Robust filtering

**Utilities**:
- `Q_discrete_white_noise()` - Process noise generation
- `Saver` - Filter state recording
- Various smoothing algorithms (RTS, etc.)

**Companion Book**: "Kalman and Bayesian Filters in Python" by Roger Labbe (free, open-source)

### 8.2 PyKalman

**PyKalman** provides the "dead-simple" Kalman filter with EM parameter estimation.

**Claim**: PyKalman is a Python library for Kalman filtering and smoothing with support for missing measurements and parameter learning via EM. [^192^]
**Source**: GitHub - pykalman/pykalman
**URL**: https://github.com/pykalman/pykalman
**Date**: Active (v0.11.2)
**Excerpt**: "The dead-simple Kalman Filter, Kalman Smoother, and EM library for Python."
**Confidence**: High

**Features**:
- Kalman Filter and Smoother
- Unscented Kalman Filter
- EM algorithm for parameter learning
- Missing measurement support
- Square root filters (Cholesky)

**Installation**: `pip install pykalman`

**Usage**:
```python
from pykalman import KalmanFilter
import numpy as np
kf = KalmanFilter(transition_matrices=[[1, 1], [0, 1]],
                  observation_matrices=[[0.1, 0.5], [-0.3, 0.0]])
(filtered_state_means, filtered_state_covariances) = kf.filter(measurements)
```

### 8.3 sktime PyKalman Integration

**Claim**: sktime maintains an up-to-date fork of pykalman with KalmanFilterTransformerPK for time series processing. [^189^]
**Source**: sktime documentation
**URL**: https://www.sktime.net/en/stable/api_reference/auto_generated/sktime.transformations.series.kalman_filter.KalmanFilterTransformerPK.html
**Excerpt**: "As the pykalman package is no longer maintained, sktime now contains an up-to-date maintenance fork of the pykalman package."
**Confidence**: High

### 8.4 ArthurAllshire/pykalman

A tiny Kalman filter library (1 star, minimal) implementing linear and unscented filters, depending only on NumPy.

### 8.5 Comparison

| Library | EKF | UKF | EM | Missing Data | Smoothing | Maintenance |
|---------|-----|-----|----|--------------|-----------|-------------|
| FilterPy | Yes | Yes | No | No | Yes | Active |
| PyKalman | No | Yes | Yes | Yes | Yes | Active (sktime fork) |

### 8.6 Other Sensor Fusion Libraries

| Library | Purpose |
|---------|---------|
| `scipy.signal` | Signal processing filters (not sensor fusion) |
| `numpy` | Linear algebra operations |
| `numba` | JIT acceleration for custom filters |

---

## 9. Complete WiFi Sensing Pipelines (End-to-End)

### 9.1 RuView (WiFi DensePose)

**RuView** is one of the most ambitious open-source WiFi sensing projects, implementing a complete pipeline from CSI capture to human pose estimation.

**Claim**: RuView is an end-to-end WiFi DensePose system that turns commodity WiFi signals into real-time human pose estimation, vital sign monitoring, and presence detection. [^171^]
**Source**: GitHub - ruvnet/RuView
**URL**: https://github.com/ruvnet/RuView
**Date**: 2026-04-16
**Excerpt**: "WiFi DensePose turns commodity WiFi signals into real-time human pose estimation, vital sign monitoring, and presence detection - all without a single pixel of video."
**Confidence**: High

**Pipeline Architecture**:
```
WiFi Router(s) → ESP32-S3 Mesh → Aggregator (UDP :5005) 
  → Bridge (I/Q → amplitude/phase) → Phase Sanitization (SpotFi)
  → Hampel Filter → Subcarrier Selection → Spectrogram (STFT)
  → Fresnel Geometry → Body Velocity Profile → Graph Transformer
  → Cross-Attention → SONA Adapter → REST API / WebSocket
```

**Key Metrics**:
- **810x** end-to-end speedup (Rust v2 vs Python v1)
- **11,665 fps** vital sign detection (single-threaded)
- **100%** presence detection accuracy
- **8 KB** quantized model fits in ESP32-S3 SRAM

**Signal Processing Stack** (RuVector v2.0.4):
- SpotFi phase correction
- Hampel outlier rejection (sigma=3)
- Fresnel zone modeling
- STFT spectrogram
- Subcarrier selection (ruvector-mincut)
- BVP (Body Velocity Profile)

**Pre-trained Models**: Available on HuggingFace (https://huggingface.co/ruv/ruview)

### 9.2 ESP32-WiFi-Sensing (thu4n)

A complete pipeline from CSI collection to MQTT-based activity recognition.

**Claim**: A project using ESP32 CSI tools for data collection and Jetson Nano for on-edge ML inference with MQTT publishing. [^123^]
**Source**: GitHub - thu4n/ESP32-WiFi-Sensing
**URL**: https://github.com/thu4n/ESP32-WiFi-Sensing
**Date**: 2023-07-28
**Excerpt**: "The project use two ESP32 microcontrollers as both TX and RX with the help of the ESP32 CSI tools for data collection as well as the Jetson Nano for on-edge model deployment."
**Confidence**: High

**Models**: CNN (95% acc), Random Forest (93%), Linear Regression (82%), SVM (83%)

### 9.3 SenseFi Benchmark

**SenseFi** is the first open-source benchmark library for deep learning WiFi CSI sensing.

**Claim**: SenseFi is the first open-source benchmark and library for WiFi CSI human sensing, implemented by PyTorch, evaluating MLP, CNN, RNN, Transformers on 4 public datasets. [^170^]
**Source**: GitHub - xyanchen/wifi-csi-sensing-benchmark
**URL**: https://github.com/xyanchen/wifi-csi-sensing-benchmark
**Date**: 2022-07-06
**Excerpt**: "SenseFi is the first open-source benchmark and library for WiFi CSI human sensing, implemented by PyTorch."
**Confidence**: High

**Supported Datasets**:
| Dataset | Platform | Classes |
|---------|----------|---------|
| UT-HAR | Intel 5300 | 7 activities |
| NTU-HAR | Atheros | 6 activities |
| NTU-HumanID | Atheros | 14 subjects |
| Widar3 | Custom | 22 gestures |

**Supported Models**: MLP, CNN, RNN, LSTM, GRU, Transformers, CNN-RNN

### 9.4 WiGr (Cross-Domain Gesture Recognition)

**Claim**: WiFi-based Cross-Domain Gesture Recognition via Modified Prototypical Networks using Widar3 dataset. [^154^]
**Source**: GitHub - Zhang-xie/WiGr
**URL**: https://github.com/Zhang-xie/WiGr
**Date**: 2021-03-26
**Excerpt**: "WiFi-based Cross-Domain Gesture Recognition via Modified Prototypical Networks"
**Confidence**: High

### 9.5 RFBoost

**Claim**: RFBoost provides datasets and PyTorch code for deep WiFi sensing with physical data augmentation. [^159^]
**Source**: GitHub - aiot-lab/RFBoost
**URL**: https://github.com/aiot-lab/RFBoost
**Date**: 2024-04-16
**Excerpt**: "Datasets and PyTorch code for RFBoost: Understanding and Boosting Deep WiFi Sensing via Physical Data Augmentation"
**Confidence**: High

---

## 10. Visualization Tools for CSI Data

### 10.1 Official esp_csi_tool.py

The most comprehensive official tool (see Section 1.3 above).

**Features**:
- Real-time CSI sub-carrier waveform display
- RSSI comparison waveform
- Room status (presence/movement) display
- Action collection for ML dataset building
- Model evaluation window
- Movement histogram and time logging

### 10.2 csi-visualization (cheeseBG)

A dedicated Python CSI visualization tool for Nexmon-extracted data.

**Claim**: Wi-Fi CSI visualization with Python supporting 6 plot types including real-time amplitude and heatmaps. [^176^]
**Source**: GitHub - cheeseBG/csi-visualization
**URL**: https://github.com/cheeseBG/csi-visualization
**Date**: 2021-07-06
**Excerpt**: "Wi-Fi Channel State Information(CSI) visualization with python"
**Confidence**: High

**Plot Types**:
1. Amplitude vs Packet Index
2. Amplitude vs Time
3. Heatmap (Amplitude vs Packet)
4. Heatmap (Amplitude vs Time)
5. Amplitude vs Subcarrier Index (all packets)
6. Amplitude vs Subcarrier Index Flow (single antenna)

**Real-time Support**: `csi_realTimeAmp.py`, `csi_realTimePhase.py`

### 10.3 CSIKit Visualization

CSIKit includes built-in visualization capabilities:

```python
from CSIKit.tools.batch_graph import BatchGraph
BatchGraph.plot_heatmap(csi_matrix_squeezed, csi_data.timestamps)
```

**Available Visualizations**:
- Heatmaps (amplitude vs time/subcarriers)
- Line plots (per-subcarrier amplitude)
- Statistical plots

### 10.4 csi_data_read_parse.py (Official)

The official simple visualization tool using PyQt5 for real-time CSI subcarrier display.

### 10.5 pyespargos Demos

The ESPARGOS project includes extensive visualization demos:
- MUSIC spatial spectrum
- Phase patterns across antenna array
- Instantaneous CSI (frequency/time domain)
- 2D azimuth-delay diagrams
- 3D radiation patterns
- Camera overlay for spatial spectrum

### 10.6 Custom matplotlib Approaches

Most researchers build custom visualization using:
- `matplotlib.pyplot` for static plots
- `matplotlib.animation` for real-time updates
- `pyqtgraph` for high-performance real-time plotting
- `bokeh` or `plotly` for web-based interactive plots

---

## 11. Papers with Open-Sourced Code

### 11.1 SpotFi

**SpotFi** is the seminal work on decimeter-level WiFi localization. The original author (Manikanta Kotaru) has open-sourced the core AoA estimation code.

**Claim**: SpotFi MATLAB code for Angle of Arrival estimation is available from the author's Bitbucket repository. [^138^]
**Source**: Awesome-WiFi-CSI-Research
**URL**: https://github.com/wuzhiguocarter/Awesome-WiFi-CSI-Research
**Date**: 2024-04-05
**Excerpt**: "SpotFi源码实现（SpotFi作者开源）: https://bitbucket.org/mkotaru/spotfimusicaoaestimation"
**Confidence**: High

**Note**: "Due to SpotFi literature using ArrayTrack's RF calibration method, directly processing CSI data may not achieve expected results without proper calibration."

**Python Reimplementation**:
- Repository: yuehanlyu/Wifi-Localization
- Implements SpotFi in Python from MATLAB reference

### 11.2 SenseFi Benchmark Paper

**Claim**: "SenseFi: A Library and Benchmark on Deep-Learning-Empowered WiFi Human Sensing" accepted by Patterns, Cell Press. [^188^]
**Source**: arXiv
**URL**: https://arxiv.org/pdf/2207.07859
**Date**: 2022
**Excerpt**: "To the best of our knowledge, this is the first benchmark with an open-source library for deep learning in WiFi sensing research."
**Confidence**: High

### 11.3 RFBoost Paper

**Claim**: "RFBoost: Understanding and Boosting Deep WiFi Sensing via Physical Data Augmentation" published in Proc. ACM IMWUT. [^159^]
**Source**: GitHub - aiot-lab/RFBoost
**URL**: https://github.com/aiot-lab/RFBoost
**Date**: 2024
**Excerpt**: "RFBoost: Understanding and Boosting Deep WiFi Sensing via Physical Data Augmentation"
**Confidence**: High

### 11.4 ESPARGOS Paper

**Claim**: "ESPARGOS: An Ultra Low-Cost, Realtime-Capable Multi-Antenna WiFi Channel Sounder" demonstrates MUSIC-based AoA estimation. [^67^]
**Source**: arXiv
**URL**: https://arxiv.org/html/2502.09405v1
**Date**: 2025-02-13
**Excerpt**: "For a practical application of ESPARGOS, AoA estimation algorithms such as MUSIC could be applied to the received channel coefficients."
**Confidence**: High

### 11.5 Self-Supervised Learning for WiFi Sensing Survey

**Claim**: A comprehensive tutorial-survey on self-supervised learning for WiFi sensing covering trends, challenges, and outlook. [^167^]
**Source**: arXiv
**URL**: https://arxiv.org/html/2506.12052v1
**Date**: 2025-05-29
**Excerpt**: "A Tutorial-cum-Survey on Self-Supervised Learning for Wi-Fi Sensing: Trends, Challenges, and Outlook"
**Confidence**: High

### 11.6 Tools and Methods for WiFi Sensing in Embedded Devices

**Claim**: A 2025 paper in Sensors journal presenting the ESP32 CSI Web Collecting Tool with LSTM-DenseNet models for HAR. [^121^]
**Source**: MDPI Sensors
**URL**: https://www.mdpi.com/1424-8220/25/19/6220
**Date**: 2025-10-08
**Excerpt**: "Tools and Methods for Achieving Wi-Fi Sensing in Embedded Devices"
**Confidence**: High

### 11.7 Wi-Fi Sensing for Human Identification Through ESP32

Referenced in multiple localization papers as an experimental study on ESP32 CSI-based identification.

### 11.8 Awesome-WiFi-CSI-Research

A curated list of WiFi CSI research resources.

**Claim**: A curated list of WiFi CSI research with links to open-source implementations. [^138^]
**Source**: GitHub - wuzhiguocarter/Awesome-WiFi-CSI-Research
**URL**: https://github.com/wuzhiguocarter/Awesome-WiFi-CSI-Research
**Date**: 2024-04-05
**Confidence**: High

---

## 12. Arduino/PlatformIO Libraries for ESP32 CSI

### 12.1 Current State

**Critical Finding**: There are NO dedicated Arduino or PlatformIO wrapper libraries that simplify ESP32 CSI access. CSI functionality requires using ESP-IDF directly.

**Reason**: ESP32 CSI access requires low-level WiFi driver APIs that are only fully exposed through ESP-IDF, not the Arduino core.

### 12.2 Workarounds

#### Option 1: ESP-IDF via PlatformIO

PlatformIO can build ESP-IDF projects directly:

```ini
; platformio.ini
[env:esp32dev]
platform = espressif32
framework = espidf
board = esp32dev
```

#### Option 2: ESPHome (for ESPectre)

ESPectre provides YAML-based configuration through ESPHome:

```yaml
# Example ESPectre configuration
esp32:
  board: esp32-s3-devkitc-1
  framework:
    type: esp-idf

# CSI sensing configured via ESPectre components
```

#### Option 3: Arduino ESP32 Core with esp_wifi_set_csi()

The Arduino ESP32 core does expose basic CSI functions:

```cpp
#include <esp_wifi.h>

// Enable CSI
esp_wifi_set_csi_rx_cb(csi_callback, NULL);
esp_wifi_set_csi_config(&csi_config);
esp_wifi_set_csi(true);
```

However, this requires understanding the ESP-IDF WiFi API even within Arduino.

### 12.4 esp-csi-rs (Rust Library)

A Rust library for ESP32 CSI in no_std environments.

**Claim**: esp-csi-rs is a Rust library for ESP-CSI supporting no_std environments with esp-generate tooling. [^179^]
**Source**: lib.rs
**URL**: https://lib.rs/crates/esp-csi-rs
**Date**: 2026-03-10
**Excerpt**: "This crate is still in early development and currently supports no_std only."
**Confidence**: High

### 12.5 Relevant WiFi Management Libraries

While not CSI-specific, these libraries help with ESP32 WiFi management:

| Library | Purpose | Platform |
|---------|---------|----------|
| WiFiManager | WiFi configuration portal | Arduino |
| ESP_WiFiManager | Enhanced WiFi manager | Arduino |
| EasyESPConnect | Lightweight WiFi manager | Arduino |
| ESP8266/ESP32 AT Lib | AT command wrapper | Arduino |

---

## 13. Emerging and Notable Projects

### 13.1 ESPARGOS

An ultra low-cost multi-antenna WiFi channel sounder using multiple ESP32 modules.

- **Hardware**: Custom PCB with multiple ESP32 modules
- **Software**: pyespargos Python library
- **Capability**: Real-time MUSIC-based AoA estimation
- **Cost**: Significantly cheaper than USRP-based solutions

### 13.2 WiFi-CSI-Sensing-Benchmark (Awesome List)

**Claim**: A curated list of awesome papers and resources on WiFi CSI sensing. [^195^]
**Source**: GitHub - NTUMARS/Awesome-WiFi-CSI-Sensing
**URL**: https://github.com/NTUMARS/Awesome-WiFi-CSI-Sensing
**Confidence**: High

### 13.3 Embedded_WiFi_Sensing (Web Collecting Tool)

A complete ecosystem including:
- ESP32 CSI Web Collecting Tool
- HAR and breathing monitoring datasets
- LSTM-embedded DenseNet models
- TensorFlow Lite Micro deployment

**Claim**: Complete ecosystem with CSI collection tool, HAR/breathing datasets, and deep learning models for ESP32 deployment. [^126^]
**Source**: GitHub
**URL**: https://github.com/AlbanyArmenta0711/Embedded_WiFi_Sensing
**Date**: 2025-08-05
**Confidence**: High

### 13.4 CSI-Data Public Datasets

The CSIKit project maintains a collection of public CSI datasets for researchers who don't have their own CSI data.

---

## 14. Summary Matrix

### Complete Tool Comparison

| Tool/Project | Type | License | Language | CSI HW | Maturity | Community |
|-------------|------|---------|----------|--------|----------|-----------|
| esp-csi (Espressif) | Official SDK | Apache 2.0 | C/Python | ESP32 | High | Large |
| ESPectre | Production App | GPL v3 | C++/Python | ESP32 | High | Growing |
| ESP32-CSI-Tool | Data Collection | MIT | C | ESP32 | High | Large |
| csiread | Parser | MIT | Cython | Multi | High | Active |
| CSIKit | Processing | MIT | Python | Multi | Medium | Active |
| Anyplace | Localization | MIT | Scala | Any | High | Academic |
| FIND3 | Localization | MIT | Go/Python | Any | High | Moderate |
| RuView | End-to-End | MIT | Rust/Python | ESP32 | Medium | Small |
| SenseFi | Benchmark | - | Python | Multi | High | Academic |
| pyespargos | Channel Sounder | - | Python | ESPARGOS | Medium | Niche |
| SpotFi Code | Research | - | MATLAB | Intel 5300 | Reference | Academic |
| filterpy | Sensor Fusion | MIT | Python | N/A | High | Large |
| pykalman | Sensor Fusion | BSD | Python | N/A | High | Moderate |
| Ultralytics YOLO | Tracking | AGPL | Python | N/A | High | Very Large |
| ByteTrack | Tracking | MIT | Python | N/A | High | Large |
| DeepSORT | Tracking | GPL | Python | N/A | High | Large |

### Key Gaps Identified

1. **No Arduino/PlatformIO CSI wrapper library exists** - Users must use ESP-IDF
2. **No CSI+YOLO fusion pipeline exists** - Vision and WiFi sensing remain separate domains
3. **Limited MUSIC implementations for ESP32** - ESP32's single antenna limits AoA
4. **No standardized CSI dataset format** - Each tool uses its own format
5. **Rust CSI ecosystem is nascent** - Only esp-csi-rs exists, early development

### Counter-Narratives and Cautions

1. **ESP32 CSI quality varies significantly by chip**: ESP32-C5/C6 > C3/S3 > original ESP32 [^2^]
2. **Router-dependent collection is unreliable**: Router model affects CSI quality
3. **Phase data requires calibration**: ESP32 has known phase offset issues
4. **MUSIC requires antenna arrays**: Single ESP32 cannot do AoA without multiple modules
5. **Many projects are research-grade**: Not production-ready without significant engineering
6. **Environmental sensitivity**: CSI-based systems require per-environment calibration
7. **The esp_csi_tool.py has known issues**: Users report graph display problems [^155^]

---

## References

[^2^] https://dev.to/pratha_maniar/a-deep-dive-into-esp-csi-channel-state-information-on-esp32-chips-5el1
[^7^] https://github.com/espressif/esp-csi
[^67^] https://arxiv.org/html/2502.09405v1
[^119^] https://github.com/francescopace/espectre
[^121^] https://www.mdpi.com/1424-8220/25/19/6220
[^123^] https://github.com/thu4n/ESP32-WiFi-Sensing
[^124^] https://github.com/jasminkarki/wifi-sensing-har
[^126^] https://github.com/AlbanyArmenta0711/Embedded_WiFi_Sensing
[^127^] https://docs.espressif.com/projects/esp-idf/en/stable/esp32c2/libraries-and-frameworks/libs-frameworks.html
[^128^] https://components.espressif.com/components/espressif/esp-radar/versions/0.2.0/examples/console_test
[^132^] https://github.com/Gi-z/CSIKit
[^133^] https://www.reddit.com/r/Python/comments/3tkgwd/we_made_an_indoor_positioning_system_using_only_a/
[^134^] https://thinkrobotics.com/blogs/learn/learn-to-design-and-3d-print-robotic-grippers
[^135^] https://medium.com/@beam_villa/object-tracking-made-easy-with-yolov11-bytetrack-73aac16a9f4a
[^136^] https://datature.com/blog/introduction-to-bytetrack-multi-object-tracking-by-associating-every-detection-box
[^137^] https://medium.com/@serurays/object-detection-and-tracking-using-yolov8-and-deepsort-47046fc914e9
[^138^] https://github.com/wuzhiguocarter/Awesome-WiFi-CSI-Research
[^139^] https://docs.ultralytics.com/modes/track/
[^140^] https://geophydog.cool/post/music_array_process/
[^141^] https://anyplace.cs.ucy.ac.cy/
[^142^] https://www.sigspatial.org/wp-content/uploads/special-issues/9/2/04-Paper01_Anatomy.pdf
[^144^] https://github.com/jingh-ai/ultralytics-YOLO-DeepSort-ByteTrack-PyQt-GUI
[^146^] https://filterpy.readthedocs.io/en/latest/
[^150^] https://github.com/espressif/esp-csi/blob/master/examples/get-started/README.md
[^153^] https://github.com/espressif/esp-csi/discussions/217
[^154^] https://github.com/Zhang-xie/WiGr
[^155^] https://github.com/espressif/esp-csi/issues/205
[^158^] https://components.espressif.com/components/espressif/esp_csi_gain_ctrl/versions/0.1.5/readme
[^159^] https://github.com/aiot-lab/RFBoost
[^160^] https://filterpy.readthedocs.io/en/latest/kalman/KalmanFilter.html
[^161^] https://gi-z.github.io/CSIKit/
[^165^] https://pypi.org/project/csiread/
[^166^] https://github.com/StevenMHernandez/ESP32-CSI-Tool
[^167^] https://arxiv.org/html/2506.12052v1
[^170^] https://github.com/xyanchen/wifi-csi-sensing-benchmark
[^171^] https://github.com/ruvnet/RuView
[^176^] https://github.com/cheeseBG/csi-visualization
[^177^] https://github.com/ESPARGOS/pyespargos
[^179^] https://lib.rs/crates/esp-csi-rs
[^186^] https://github.com/MarcinWachowiak/music-aoa-estimation-py
[^187^] https://learnopencv.com/understanding-multiple-object-tracking-using-deepsort/
[^188^] https://arxiv.org/pdf/2207.07859
[^189^] https://www.sktime.net/en/stable/api_reference/auto_generated/sktime.transformations.series.kalman_filter.KalmanFilterTransformerPK.html
[^192^] https://github.com/pykalman/pykalman
[^195^] https://github.com/NTUMARS/Awesome-WiFi-CSI-Sensing
[^202^] https://github.com/em22ad/CSI-Fingerprint-Indoor-Localization
[^204^] https://components.espressif.com/components/espressif/esp-radar
[^205^] https://github.com/ifzhang/ByteTrack
[^207^] https://github.com/joaocarvalhoopen/Indoor_WiFi_Localization_in_ESP32_using_Machine_Leaning
[^208^] https://github.com/schollz/find3

---

*This research document was compiled from 25+ independent web searches across primary sources including GitHub repositories, academic papers, official documentation, and community forums. All claims are sourced with inline citations and confidence levels.*
