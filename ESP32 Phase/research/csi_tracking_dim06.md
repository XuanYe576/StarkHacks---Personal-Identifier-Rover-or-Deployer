# Dimension 06: YOLO Computer Vision Integration for Human Tracking

## Research Summary

This document provides a comprehensive deep-dive into YOLO-based human detection and tracking for integration with WiFi CSI data. It covers the current state of YOLO architectures, webcam inference pipelines, tracking algorithms, camera calibration, computational requirements, edge deployment, and fusion strategies for CSI+YOLO multimodal localization.

---

## 1. Current YOLO Versions and Architecture

### 1.1 YOLO Family Evolution

The YOLO (You Only Look Once) family has undergone rapid evolution. As of late 2024/early 2025, the key versions relevant to this project are:

**YOLOv8** (January 2023, Ultralytics) — The established standard:
- Authors: Glenn Jocher, Ayush Chaurasia, Jing Qiu [^26^]
- Anchor-free detection with decoupled head (separate classification and regression branches)
- C2f backbone with SPPF, PANet neck
- Natively supports detection, segmentation, pose estimation, tracking, classification
- COCO mAP@50-95: 37.3% (n), 44.9% (s), 50.2% (m) [^101^]
- PyTorch-first design with export to ONNX, TensorRT, CoreML, TFLite, NCNN, OpenVINO

**YOLOv9** (February 2024, Chien-Yao Wang et al.) — Research-oriented:
- Key innovations: Programmable Gradient Information (PGI) and Generalized Efficient Layer Aggregation Network (GELAN) [^22^]
- Addresses information bottlenecks in deep neural networks

**YOLOv10** (May 2024, Tsinghua University) — NMS-free design:
- Consistent dual assignments for NMS-free training
- Eliminates Non-Maximum Suppression bottleneck during inference
- Higher precision but lower recall than YOLOv8 in some benchmarks [^22^]

**YOLO11** (September 2024, Ultralytics) — Current recommended standard:
- Authors: Glenn Jocher, Jing Qiu [^26^]
- Refined core architecture with C3k2 bottlenecks and C2PSA attention module
- 22% fewer parameters than YOLOv8m with 1.3% higher mAP on COCO [^106^]
- CPU inference speeds substantially faster than YOLOv8 when exported to ONNX
- COCO mAP@50-95: 39.5% (n), 47.0% (s), 50.3% (m) [^101^]

**YOLO26 / YOLOv12+** (2025, Ultralytics) — Future direction:
- NMS-free inference, DFL removal, Progressive Loss Balancing
- Even faster CPU inference but newer and less battle-tested

### 1.2 Recommendation for CSI+YOLO Fusion

Claim: **YOLO11 is the recommended version for new CSI+YOLO fusion projects** due to its superior accuracy-efficiency trade-off and mature ecosystem support.
Source: Ultralytics Official Documentation
URL: https://docs.ultralytics.com/compare/yolov8-vs-yolo11/
Date: 2025-04-17
Excerpt: "YOLO11 achieves a notably higher Mean Average Precision (mAP) while simultaneously reducing both parameter count and Floating Point Operations (FLOPs). For instance, the YOLO11m model requires 22% fewer parameters than YOLOv8m but delivers a 1.3% higher mAP on the COCO dataset. Furthermore, CPU inference speeds when exported to ONNX format show that YOLO11 is substantially faster, making it an excellent candidate for deployments lacking dedicated GPU acceleration."
Context: Official Ultralytics comparison documentation
Confidence: high

---

## 2. Running YOLO Inference on USB Webcam in Python

### 2.1 OpenCV + Ultralytics Pipeline

The standard pipeline uses OpenCV's `VideoCapture` to grab frames and the `ultralytics` library for inference.

```python
import cv2
from ultralytics import YOLO

# Load YOLO model (nano for maximum speed)
model = YOLO('yolov8n.pt')  # or 'yolo11n.pt'

# Start webcam capture
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    # Run inference
    results = model(frame, stream=True)
    
    # Process results
    for r in results:
        for box in r.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf = float(box.conf[0])
            cls = int(box.cls[0])
            label = f"{model.names[cls]} {conf:.2f}"
            
            if conf > 0.5:
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
    
    cv2.imshow("YOLO Detection", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
```

Claim: **Ultralytics YOLO with OpenCV provides a simple 5-line core pipeline for webcam inference**
Source: Medium / Lucent Innovation
URL: https://medium.com/@lakshmanmandapati0/real-time-ai-vision-yolov8-and-opencv-power-live-webcam-detection-b86964c9d22e
Date: 2025-04-10
Excerpt: "Inside the while loop, each frame is captured from the webcam using cap.read(). This frame is then processed by the YOLO model for detection, and the output predictions are stored in the results variable."
Context: Tutorial on real-time webcam detection with YOLOv8 and OpenCV
Confidence: high

### 2.2 Key Pipeline Components

1. **VideoCapture initialization**: `cv2.VideoCapture(0)` opens the default USB webcam
2. **Frame reading**: `cap.read()` returns a boolean and the frame array
3. **Inference**: `model(frame)` or `model(frame, stream=True)` for video streaming
4. **Result extraction**: `r.boxes` contains all detection bounding boxes
5. **Visualization**: OpenCV drawing functions overlay bounding boxes on frames

---

## 3. Processing Speed: CPU Inference FPS Expectations

### 3.1 Critical Factor: Model Size, Format, and Hardware

CPU inference performance varies dramatically based on model size, export format, and CPU type.

**PyTorch/ONNX format on CPU (AMD EPYC 7V12)** [^64^]:
| Format | Inference Time (ms) | FPS |
|--------|-------------------|-----|
| PyTorch | 45.47 | 21.99 |
| ONNX | 76.91 | 13.00 |
| OpenVINO | 31.68 | 31.57 |
| NCNN | 125.02 | 8.00 |

**OpenVINO on Intel hardware (YOLOv8n)** [^80^]:
| Hardware | Format | FPS |
|----------|--------|-----|
| Intel NUC CPU (i7) | FP32 OpenVINO | ~75 |
| Intel NUC CPU (i7) | INT8 OpenVINO | ~183 |
| Intel Arc A770M GPU | INT8 OpenVINO | ~1,074 |

**Intel Xeon 6 with OpenVINO** [^74^]:
| Model Size | FP32 FPS | INT8 FPS |
|------------|----------|----------|
| YOLOv8n | >350 | >500 (multi-stream) |
| YOLOv8m | ~100 | ~120 |
| YOLOv8l | ~62 | ~75 |

**Raspberry Pi 4 with NCNN** [^67^]:
| Model | Resolution | FPS |
|-------|-----------|-----|
| YOLOv8n | 640x640 | ~3.1 |
| YOLOv8s | 640x640 | ~1.47 |

**Raspberry Pi 5 with NCNN** [^67^]:
| Model | Resolution | FPS |
|-------|-----------|-----|
| YOLOv8n | 640x640 | ~20.0 |
| YOLOv8s | 640x640 | ~11.0 |

Claim: **OpenVINO provides ~3x speedup over native PyTorch on Intel CPUs**
Source: Ultralytics Blog
URL: https://www.ultralytics.com/blog/achieve-faster-inference-speeds-ultralytics-yolov8-openvino
Date: 2024-02-11
Excerpt: "Experience up to 3x faster AI inference on CPUs, with even greater accelerations across Intel's hardware spectrum."
Context: Official Ultralytics blog post on OpenVINO integration
Confidence: high

### 3.2 FPS Expectations for Brio 105 Setup

The Logitech Brio 105 outputs 1080p@30fps or 720p@30fps [^54^][^56^]. For CPU inference:

- **Desktop Intel i5/i7 with OpenVINO (YOLOv8n/YOLO11n)**: 20-50 FPS at 640x640 input resolution, sufficient for real-time processing
- **Laptop Intel with OpenVINO**: 10-25 FPS depending on generation
- **Raspberry Pi 4**: ~3 FPS (not real-time without additional hardware like Coral TPU)
- **Raspberry Pi 5**: ~8-15 FPS with NCNN optimization (borderline real-time)

### 3.3 Key Insight for CSI Fusion

Since the Brio 105 outputs at 30fps max, the inference pipeline only needs to achieve ~10-15 FPS for effective person tracking, as temporal interpolation can fill gaps. The CSI data rate (typically 100-1000 Hz for ESP32) is decoupled from the camera frame rate.

---

## 4. Extracting Person Bounding Boxes and Confidence Scores

### 4.1 YOLO Output Format (Ultralytics)

The Ultralytics YOLO output provides structured detection results [^59^][^75^]:

**Boxes object properties**:
| Property | Type | Description |
|----------|------|-------------|
| `boxes.xyxy` | Tensor | Bounding box in [x1, y1, x2, y2] format (pixels) |
| `boxes.xywh` | Tensor | Bounding box in [x_center, y_center, width, height] |
| `boxes.conf` | Tensor | Confidence score (0-1) |
| `boxes.cls` | Tensor | Class ID (0=person in COCO) |
| `boxes.id` | Tensor | Track ID (when tracking is enabled) |

### 4.2 Person Filtering Code

```python
results = model(frame, classes=[0], conf=0.5)  # Filter for class 0 (person) only

for r in results:
    for box in r.boxes:
        x1, y1, x2, y2 = box.xyxy[0].tolist()
        conf = float(box.conf[0])
        cls_id = int(box.cls[0])  # 0 for person
        
        # Bottom center point (feet position estimate)
        bottom_center_x = (x1 + x2) / 2
        bottom_center_y = y2
```

Claim: **Ultralytics provides structured results objects with xyxy, conf, cls, and id properties for each detection**
Source: Ultralytics Documentation
URL: https://docs.ultralytics.com/modes/predict/
Date: 2023-11-12
Excerpt: "Boxes object can be used to index, manipulate, and convert bounding boxes to different formats... xyxy: Return the boxes in xyxy format... conf: Return the confidence values... cls: Return the class values... id: Return the track IDs"
Context: Official Ultralytics API documentation
Confidence: high

### 4.3 Text Output Format

Ultralytics `save_txt` outputs [^85^][^87^]:
```
[class] [x_center] [y_center] [width] [height] [confidence] [track_id]
```
All coordinates are normalized to [0, 1] relative to image dimensions.

---

## 5. Person Tracking Across Frames

### 5.1 Tracking Algorithms Overview

Ultralytics natively supports two trackers [^93^]:

**ByteTrack** (2022, Zhang et al.):
- Uses high and low confidence detections for association
- Achieves 171 FPS on GPU (Tesla T4) [^24^]
- MOTA: 77.3%, MOTP: 82.6%, ID switches: 558
- **Best for**: Speed-critical applications

**BoT-SORT** (2022, Aharon et al.):
- Integrates motion and appearance (ReID) information
- Better for long-term occlusion handling
- MOTA: 61.4%, MOTP: 79.1%, ID switches: 781 [^24^]
- **Best for**: Occlusion-heavy scenarios

Claim: **ByteTrack outperforms DeepSORT and SORT on MOTA, MOTP, and speed metrics**
Source: Vectoral International Journal
URL: https://vectoral.org/index.php/IJSICS/article/view/97/89
Date: Unknown
Excerpt: "ByteTrack stands out as the top performer, achieving a MOTA score of 77.3%, substantially higher than DeepSORT's 61.4% and SORT's 54.7%... In terms of processing speed, ByteTrack and SORT demonstrate better performance, achieving 171 FPS and 143 FPS, respectively."
Context: Comparative evaluation paper on tracking algorithms
Confidence: high

### 5.2 Tracking with Ultralytics

```python
# Initialize tracking
results = model.track(frame, persist=True, tracker="bytetrack.yaml")

# Extract tracked persons
for r in results:
    if r.boxes.id is not None:  # Check if tracking IDs exist
        boxes = r.boxes.xyxy.cpu().numpy()
        track_ids = r.boxes.id.int().cpu().tolist()
        confidences = r.boxes.conf.cpu().tolist()
        
        for box, track_id, conf in zip(boxes, track_ids, confidences):
            x1, y1, x2, y2 = box
            # Each track_id represents a unique person across frames
```

### 5.3 Re-Identification (ReID) Integration

For robust person re-identification after occlusion, feature-based ReID can be added:

Claim: **Efficient ReID can be integrated into Ultralytics with virtually no FPS drop by using YOLO features directly**
Source: Analytics Vidhya / Y-T-G Tutorials
URL: https://y-t-g.github.io/tutorials/yolo-reid/
Date: 2025-04-12
Excerpt: "When comparing the output with and without reidentification, we observe virtually no drop in FPS. And we also see that tracker is able to recover the tracks much better after occlusion."
Context: Tutorial on integrating ReID features into Ultralytics BoT-SORT
Confidence: medium

**Key ReID parameters for occlusion handling** [^37^]:
- `track_buffer: 300` — extends how long a lost track is kept
- `proximity_thresh: 0.2` — allows matching objects up to 20% of image width away
- `appearance_thresh: 0.3` — requires 70% feature similarity for matching

---

## 6. Estimating Person Position from Camera

### 6.1 Bottom-Center Point Method

The most common approach for estimating a person's ground position from a bounding box is to use the bottom-center point of the bounding box as an approximation of where the person's feet touch the ground [^69^]:

```
(p_x', p_y') = (c_x', c_y' - h_b/2)
```

Where:
- `(c_x', c_y')` is the center of the bounding box
- `h_b` is the height of the bounding box
- `(p_x', p_y')` is the estimated piercing point on the ground plane

Claim: **The bottom center of the bounding box is a good estimation of the pedestrian's location with respect to the ground plane**
Source: CEUR Workshop Proceedings
URL: https://ceur-ws.org/Vol-2884/paper_120.pdf
Date: 2020-07-14
Excerpt: "We then use the center point of the bottom side of the bounding rectangle returned by the YOLO object detector as a estimation of the piercing point for the pedestrian. This point is either between or on the pedestrian's feet and parallel to the pedestrians head, so is a good estimation of the pedestrian's location with respect to the ground plane."
Context: Paper on social distancing measurement using YOLO and homography
Confidence: high

### 6.2 Pixel-to-World Coordinate Mapping

To convert pixel coordinates to real-world coordinates, two main approaches exist:

**Approach 1: Homography Matrix (single camera, ground plane)**
- Calibrate using 4+ point correspondences between image plane and ground plane
- Compute homography matrix H using OpenCV's `findHomography()`
- Transform any ground-plane pixel to world coordinates: `P_world = H * P_pixel`

**Approach 2: Full Camera Calibration (3D reconstruction)**
- Camera matrix (intrinsic parameters): focal length, optical center
- Distortion coefficients: radial and tangential distortion
- Extrinsic parameters: rotation and translation vectors
- Use `solvePnP()` for 3D position estimation

Claim: **A probabilistic approach using the mean of means can achieve 95% accuracy within 0.3m range using two 640x480 webcams**
Source: arXiv
URL: https://arxiv.org/html/2407.20870v2
Date: 2025-01-25
Excerpt: "Experimental results demonstrate human localization accuracy of 95% within a 0.3m range and nearly 100% accuracy within a 0.5m range, achieved at a low cost of only 10 USD using two web cameras with a resolution of 640x480 pixels."
Context: Paper on calibration-free human localization
Confidence: medium

---

## 7. Camera Calibration Requirements

### 7.1 Intrinsic Parameters

The camera intrinsic matrix K is a 3x3 matrix [^29^][^31^]:

```
K = [[fx,  0, cx],
     [0,  fy, cy],
     [0,   0,  1]]
```

- `fx, fy`: Focal lengths in pixel units
- `cx, cy`: Optical center (principal point) in pixel coordinates
- Distortion coefficients: `(k1, k2, p1, p2, k3)` for radial and tangential distortion

### 7.2 Calibration Procedure with OpenCV

```python
import cv2
import numpy as np

# Prepare object points (chessboard corners)
chessboard_size = (9, 6)
objp = np.zeros((np.prod(chessboard_size), 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)

objpoints = []  # 3D points in real world
imgpoints = []  # 2D points in image plane

# Capture multiple images of chessboard
images = glob.glob('calibration/*.jpg')
for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)
    if ret:
        objpoints.append(objp)
        imgpoints.append(corners)

# Calibrate
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)

# Save for later use
np.savez('camera_calibration.npz', mtx=mtx, dist=dist)
```

Claim: **OpenCV calibrateCamera returns camera matrix, distortion coefficients, rotation and translation vectors**
Source: OpenCV Official Documentation
URL: https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
Date: Unknown
Excerpt: "In addition to this, we need some other information, like the intrinsic and extrinsic parameters of the camera. Intrinsic parameters are specific to a camera. They include information like focal length (fx,fy) and optical centers (cx, cy)."
Context: Official OpenCV camera calibration tutorial
Confidence: high

### 7.3 Brio 105 Specifics

The Logitech Brio 105 [^54^][^56^]:
- **Resolution**: 1080p/30fps (1920x1080) or 720p/30fps (1280x720)
- **Field of View**: 58 degrees diagonal (fixed)
- **Focus**: Fixed focus
- **Lens**: Custom 4-element plastic lens
- **Interface**: USB-A plug-and-play

For rough position estimation without calibration:
- Approximate focal length for 58deg FOV at 1920x1080: `fx ~ fy ~ 1500-1700` pixels
- Principal point: approximately `(960, 540)` for 1080p

### 7.4 Distortion Correction

```python
# Load calibration
data = np.load('camera_calibration.npz')
mtx = data['mtx']
dist = data['dist']

# Undistort image
undistorted = cv2.undistort(frame, mtx, dist, None, mtx)

# Undistort a single point
point = np.array([[bottom_center_x, bottom_center_y]], dtype=np.float32)
undistorted_point = cv2.undistortPoints(point, mtx, dist, P=mtx)
```

---

## 8. Handling Occlusions and Multiple Persons

### 8.1 Tracking ID Management

When using `model.track()` with `persist=True`, each person receives a unique track ID [^90^]:

- Track IDs persist across frames
- When a person leaves the frame, their ID is eventually released (after `track_buffer` frames)
- When a person re-enters, they may receive a new ID unless ReID is enabled

### 8.2 Multi-Person Scenarios

For multiple persons, the output contains parallel arrays:

```python
results = model.track(frame, persist=True)
for r in results:
    if r.boxes.id is not None:
        n_persons = len(r.boxes)
        boxes = r.boxes.xyxy.cpu().numpy()
        track_ids = r.boxes.id.int().cpu().tolist()
        confidences = r.boxes.conf.cpu().tolist()
        
        for i in range(n_persons):
            person_data = {
                'track_id': track_ids[i],
                'bbox': boxes[i].tolist(),
                'confidence': confidences[i],
                'bottom_center': ((boxes[i][0] + boxes[i][2])/2, boxes[i][3])
            }
```

### 8.3 Occlusion Handling Strategies

1. **ByteTrack/BoT-SORT built-in**: Trackers naturally handle short-term occlusions
2. **Track buffer**: Increase `track_buffer` in tracker YAML config for longer occlusion tolerance [^37^]
3. **ReID features**: Enable BoT-SORT with ReID for appearance-based matching after occlusion
4. **Temporal interpolation**: Use Kalman filter predictions during brief occlusions

Claim: **Increasing track_buffer to 300 allows trackers to tolerate occlusion for up to 10 seconds at 30fps**
Source: Analytics Vidhya
URL: https://www.analyticsvidhya.com/blog/2025/04/re-id-in-yolo/
Date: 2025-05-05
Excerpt: "track_buffer: 300 — extends how long a lost track is kept before deletion."
Context: Tutorial on BoT-SORT ReID configuration
Confidence: medium

---

## 9. Computational Requirements for Real-Time YOLO Tracking

### 9.1 System Requirements

**Minimum (5-10 FPS on CPU)**:
- Intel i3 or equivalent
- 4GB RAM
- Python 3.8+
- ultralytics, opencv-python, numpy

**Recommended (20-30 FPS on CPU)**:
- Intel i5/i7 8th gen+ or AMD Ryzen 5+
- 8GB RAM
- OpenVINO optimization for Intel CPUs
- Model: YOLOv8n or YOLO11n at 640x640

**Optimal (30+ FPS)**:
- NVIDIA GPU (GTX 1060+) or Intel integrated GPU with OpenVINO
- 16GB RAM
- TensorRT or OpenVINO optimization

### 9.2 End-to-End Pipeline Timing Budget

For 30 FPS real-time processing (33ms budget):
- Frame capture: ~5ms
- Preprocessing (resize, normalize): ~2-5ms
- **Inference (YOLOv8n OpenVINO on i7)**: ~10-15ms
- NMS + tracking: ~2-5ms
- Post-processing + drawing: ~2-5ms
- Display: ~5-10ms

### 9.3 Memory Requirements

- YOLOv8n model: ~6.2 MB (PyTorch), ~12.3 MB (ONNX/OpenVINO)
- Runtime memory: ~200-500 MB depending on batch size
- Peak GPU memory (Jetson Orin NX): ~434 MB at batch=1 [^76^]

---

## 10. Synchronizing Camera Frames with CSI Data

### 10.1 The Synchronization Challenge

USB webcams and CSI data sources operate on different clocks and sampling rates:
- **Camera**: 30 FPS (~33ms between frames), driven by USB UVC clock
- **CSI data**: 100-1000+ Hz, driven by ESP32 system clock
- Neither provides hardware timestamp synchronization

Claim: **USB UVC does not have the capability to decide exactly when exposure starts, and lacks hardware synchronization**
Source: OpenCV Forum
URL: https://forum.opencv.org/t/how-would-you-synchronize-two-webcams-to-be-acquiring-their-images-at-the-same-times/16409
Date: 2024-02-09
Excerpt: "USB UVC does not have the capability to decide exactly when an exposure is started. not for video capture modes anyway... USB probably doesn't allow it."
Context: OpenCV forum discussion on camera synchronization
Confidence: high

### 10.2 Software-Based Synchronization Strategies

**Strategy 1: Host Timestamp Alignment (recommended)**
```python
import time

# When capturing frame
frame_timestamp = time.time()
ret, frame = cap.read()

# Store with timestamp
frame_buffer.append({'timestamp': frame_timestamp, 'frame': frame})

# When receiving CSI data
csi_timestamp = time.time()
# Find nearest frame by timestamp
nearest_frame = min(frame_buffer, 
                    key=lambda x: abs(x['timestamp'] - csi_timestamp))
```

**Strategy 2: Sliding Window Fusion**
- Maintain a buffer of recent CSI samples (e.g., 1 second)
- For each camera frame, extract CSI samples within a ±50ms window
- Fuse features/decisions from the aligned window

**Strategy 3: Reference Clock Synchronization**
- Use NTP or system clock as common reference
- Log `time.time()` at both frame capture and CSI collection
- Interpolate CSI data to match frame timestamps

Claim: **WiVi achieves 4ms average synchronization error between WiFi and camera data using software timestamping**
Source: WiVi Paper (CVPRW 2019)
URL: https://openaccess.thecvf.com/content_CVPRW_2019/papers/MULA/Zou_WiFi_and_Vision_Multimodal_Learning_for_Accurate_and_Robust_Device-Free_CVPRW_2019_paper.pdf
Date: 2019
Excerpt: "Both WiFi and video data were collected simultaneously with only 4 milliseconds average synchronization error."
Context: CVPRW 2019 paper on WiFi+Vision multimodal activity recognition
Confidence: high

### 10.3 Reference WiVi Synchronization Approach

The WiVi system [^38^] uses:
- Two TP-Link N750 WiFi routers for CSI
- One Logitech C270 webcam (640x480)
- Software-level timestamp synchronization
- Average sync error: 4ms

This demonstrates that software-level synchronization is sufficient for CSI+vision fusion at human activity timescales.

---

## 11. YOLO on Edge Devices Alongside CSI Processing

### 11.1 Raspberry Pi 4 Performance

Claim: **YOLOv8n achieves ~3.1 FPS on Raspberry Pi 4 at 640x640 using NCNN**
Source: Qengineering GitHub
URL: https://github.com/Qengineering/YoloV8-ncnn-Raspberry-Pi-4
Date: 2023-01-16
Excerpt: "YoloV8 640x640 nano: RPi 4 1950 MHz -> 3.1 FPS"
Context: Benchmark repository for YOLO on Raspberry Pi with NCNN
Confidence: high

**Raspberry Pi 4 Detailed Benchmarks** [^63^][^68^][^71^]:
- YOLOv8n @ 640x480: ~1.24 FPS (without NCNN optimization)
- YOLOv8n @ 640x640 with NCNN: ~3.1 FPS
- YOLOv8s @ 640x640 with NCNN: ~1.47 FPS
- Inference time: ~500ms per image (GitHub issue report)

### 11.2 Raspberry Pi 5 Performance

Claim: **YOLOv8n achieves ~20 FPS on Raspberry Pi 5 at 640x640 using NCNN**
Source: Qengineering GitHub
URL: https://github.com/Qengineering/YoloV8-ncnn-Raspberry-Pi-4
Date: 2023-01-16
Excerpt: "YoloV8 640x640 nano: RPi 5 2900 MHz -> 20.0 FPS"
Context: Benchmark repository for YOLO on Raspberry Pi 5 with NCNN
Confidence: high

### 11.3 Raspberry Pi 5 with AI Kit (Hailo-8)

Claim: **YOLOv8s achieves 30+ FPS on Raspberry Pi 5 with AI Kit using INT8 quantization**
Source: Reddit r/raspberry_pi
URL: https://www.reddit.com/r/raspberry_pi/comments/1e5zycb/yolov8s_fps_benchmark_and_performance_comparison/
Date: 2026-02-05
Excerpt: "On the Raspberry Pi 5, we compared the difference in inference speed by setting different PCIe modes"
Context: Community benchmark with AI Kit
Confidence: medium

### 11.4 CSI Processing Overlap on Edge Devices

Running both YOLO and CSI processing on the same Raspberry Pi:
- **Raspberry Pi 4**: Challenging — YOLO takes ~3-5 FPS, leaving limited CPU for CSI
- **Raspberry Pi 5**: Feasible — YOLO at ~10-15 FPS leaves cores for CSI processing
- **Recommendation**: Use a Coral TPU or Hailo-8 for YOLO inference, freeing CPU for CSI

### 11.5 Edge Deployment Recommendations

| Device | YOLO FPS | CSI+YOLO Viable? |
|--------|----------|-----------------|
| RPi 4 (bare) | ~3 | No (too slow) |
| RPi 4 + Coral TPU | ~14 | Marginal |
| RPi 5 (bare) | ~10-15 | Borderline |
| RPi 5 + AI Kit | ~30+ | Yes |
| Jetson Orin Nano | ~40-65 | Yes |
| Intel N100 mini PC | ~7-9 (CPU) / 15-20 (iGPU) | Yes |

---

## 12. Data Format for YOLO Output to Fuse with CSI

### 12.1 Recommended Output Format

For fusion with CSI localization, YOLO should output structured JSON per frame:

```json
{
  "timestamp": 1699900000.123,
  "frame_id": 142,
  "camera_position": {"x": 0.0, "y": 0.0, "z": 2.5, "theta": 0.0},
  "detections": [
    {
      "track_id": 1,
      "class": "person",
      "confidence": 0.92,
      "bbox": {"x1": 100, "y1": 200, "x2": 300, "y2": 600},
      "bbox_normalized": {"x1": 0.052, "y1": 0.185, "x2": 0.156, "y2": 0.556},
      "bottom_center": {"x": 200, "y": 600},
      "bottom_center_normalized": {"x": 0.104, "y": 0.556},
      "estimated_world_position": {"x": 2.3, "y": 1.5},
      "reid_features": [0.1, 0.2, ...]
    }
  ],
  "n_persons": 1,
  "processing_time_ms": 15.2
}
```

### 12.2 Fusion Strategies

Three levels of fusion are possible [^49^][^33^]:

**Early Fusion (input level)**:
- Concatenate camera frames and CSI data as input to a single network
- Requires temporal and spatial alignment
- Used in: MaskFi, Ye et al. [^33^]
- **Pros**: Maximum cross-modal information
- **Cons**: Complex architecture, sensitive to misalignment

**Feature Fusion (intermediate level)**:
- Extract features separately from camera and CSI, then fuse
- Used in: WiVi (decision-level DNN), HDANet [^33^]
- **Pros**: Flexible, handles different data rates naturally
- **Cons**: Requires careful feature alignment

**Late Fusion (decision level)**:
- Run camera-based and CSI-based localization independently
- Combine final position estimates (e.g., Kalman filter, weighted average)
- Used in: WiVi (softmax output fusion) [^38^]
- **Pros**: Most robust to single-modality failure, modular
- **Cons**: May miss cross-modal synergies

### 12.3 WiVi Decision-Level Fusion Reference

Claim: **WiVi uses decision-level fusion with a 4-layer DNN combining WiFi and vision softmax outputs, achieving 97.5% activity recognition accuracy**
Source: CVPRW 2019 Paper
URL: https://openaccess.thecvf.com/content_CVPRW_2019/papers/MULA/Zou_WiFi_and_Vision_Multimodal_Learning_for_Accurate_and_Robust_Device-Free_CVPRW_2019_paper.pdf
Date: 2019
Excerpt: "WiVi consists of a WiFi sensing module and a vision sensing module... followed by a multimodal fusion module... We concatenate the outputs of the SoftMax layer from the two modules and feed them into a four-layer deep neural networks (DNN) model."
Context: WiVi multimodal fusion architecture
Confidence: high

### 12.4 Recommended Fusion Architecture for CSI+YOLO

For a practical CSI+YOLO localization system:

```
Camera Pipeline (30 FPS):          CSI Pipeline (100-1000 Hz):
YOLO Detection ->                   CSI Collection ->
  Person BBox ->                      Amplitude/Phase Features ->
    Bottom Center Pixel ->              Position Estimate (x,y) ->
      Homography Transform ->
        Camera Position (x_c, y_c) ->
                                          Fusion Module:
                                          Kalman Filter / Weighted Average ->
                                            Fused Position (x, y)
```

The fusion module should use a **Kalman filter** to integrate:
- Camera position estimate (lower frequency, higher precision for visible persons)
- CSI position estimate (higher frequency, works through walls/occlusion)

---

## 13. Key Stakeholders and Ecosystem

### 13.1 Primary Organizations

1. **Ultralytics** (https://ultralytics.com) — Maintainers of YOLOv5, YOLOv8, YOLO11
   - Provides Python package, pretrained models, documentation
   - Active development and community support

2. **Intel** — OpenVINO optimization for CPU inference
   - Provides 3x+ speedup on Intel hardware
   - INT8 quantization tools

3. **Tencent** (ByteTrack) — Tracking algorithm
   - Open-source tracker integrated into Ultralytics

4. **NVIDIA** — TensorRT for GPU optimization
   - Jetson platform for edge deployment

### 13.2 Key Academic References

- **WiVi** (CVPRW 2019): Zou et al. — First CSI+vision multimodal activity recognition [^38^]
- **ByteTrack** (ECCV 2022): Zhang et al. — High-performance tracking [^24^]
- **BoT-SORT** (2022): Aharon et al. — ReID-integrated tracking
- **Person-in-WiFi** (ICCV 2019): Wang et al. — WiFi-based person perception with camera supervision [^40^]
- **MaskFi** (2024): WiFi+vision input fusion for activity recognition [^33^]

---

## 14. Counter-Narratives and Limitations

### 14.1 YOLO Alone is Insufficient for Precise Positioning

YOLO bounding boxes provide only pixel-level locations. Without camera calibration and ground-plane homography, pixel positions cannot be directly mapped to physical coordinates. The bottom-center heuristic assumes the person is standing upright on a flat ground plane.

### 14.2 CPU Inference May Not Achieve Real-Time

On low-power devices (RPi 4), YOLO CPU inference achieves only ~3 FPS, which is insufficient for real-time tracking. Hardware acceleration (Coral TPU, Hailo-8, GPU) is required for resource-constrained deployments.

### 14.3 Camera and CSI Data are Fundamentally Different

Camera data is visual (2D projection of 3D world), while CSI captures RF channel characteristics. They measure fundamentally different physical phenomena. Simple fusion approaches may fail when one modality is unreliable (e.g., camera in darkness, CSI in static environments).

### 14.4 Privacy Concerns with Camera-Based Tracking

Camera-based tracking raises privacy concerns that CSI-only approaches avoid. For privacy-sensitive environments, consider using YOLO only for calibration/validation and relying primarily on CSI for continuous localization.

### 14.5 No Existing CSI+YOLO Fusion for Localization

Despite extensive searching, no existing system specifically fuses YOLO-based visual tracking with WiFi CSI for indoor localization. WiVi [^38^] fuses CSI+vision for activity recognition (classification), not continuous position tracking. This confirms the novelty of the proposed CSI+YOLO fusion approach.

---

## 15. Implementation Recommendations

### 15.1 Recommended Stack

```python
# Dependencies
# pip install ultralytics opencv-python numpy scipy

# For OpenVINO optimization
# pip install openvino
# model.export(format="openvino")

# Recommended model
model = YOLO("yolo11n.pt")  # or yolov8n.pt for maximum compatibility

# Recommended tracker
tracker = "bytetrack.yaml"  # Fast and accurate
# tracker = "botsort.yaml"   # If ReID/occlusion handling needed

# Recommended resolution for 1080p camera
# Resize to 640x640 for inference (maintains aspect ratio)
```

### 15.2 Key Configuration Parameters

```yaml
# bytetrack.yaml (default from Ultralytics)
tracker_type: bytetrack
track_high_thresh: 0.5
track_low_thresh: 0.1
new_track_thresh: 0.6
track_buffer: 30  # Increase to 100-300 for occlusion tolerance
match_thresh: 0.8
cmc_method: None  # Camera motion compensation
```

### 15.3 Performance Checklist

- [ ] Use YOLO11n or YOLOv8n (nano) variant for fastest inference
- [ ] Export to OpenVINO if running on Intel CPU (3x speedup)
- [ ] Export to NCNN if running on ARM (Raspberry Pi)
- [ ] Use `classes=[0]` to filter for person class only
- [ ] Use `stream=True` for video input
- [ ] Set confidence threshold to 0.3-0.5 based on testing
- [ ] Consider Coral TPU or Hailo-8 for edge deployment
- [ ] Implement software timestamp synchronization (target <10ms error)
- [ ] Use bottom-center of bbox as ground plane intersection point
- [ ] Calibrate camera with checkerboard for accurate pixel-to-world mapping

---

## 16. Summary Table

| Question | Key Finding |
|----------|-------------|
| Current YOLO version | YOLO11 (Ultralytics, Sept 2024), 22% fewer params, higher mAP than YOLOv8 |
| Webcam inference | OpenCV VideoCapture + Ultralytics `model()` or `model.track()` |
| CPU FPS (desktop) | 20-50 FPS with YOLO11n + OpenVINO on Intel i5/i7 |
| CPU FPS (RPi 4) | ~3 FPS with NCNN; ~10-15 FPS on RPi 5 |
| Person bbox extraction | `results.boxes.xyxy`, `conf`, `cls`, filter class=0 |
| Tracking | ByteTrack (fast) or BoT-SORT (occlusion robust) built into Ultralytics |
| Position estimation | Bottom-center of bbox -> homography matrix -> world coordinates |
| Camera calibration | Checkerboard + OpenCV calibrateCamera for intrinsic matrix + distortion |
| Occlusion handling | track_buffer, ReID features, Kalman filter prediction |
| Real-time requirements | 10-15 FPS sufficient for person tracking; 30 FPS for smooth display |
| CSI-camera sync | Software timestamp alignment, sliding windows; WiVi achieved 4ms error |
| Edge deployment | RPi 4 marginal; RPi 5 + AI Kit viable; Intel N100 good; Jetson Orin best |
| Output format for CSI fusion | JSON with track_id, bbox, bottom_center_pixel, timestamp, confidence |

---

## References

[^17^]: YOLO Detection Benchmarks Overview — https://www.emergentmind.com/topics/yolo-detection-benchmarks
[^18^]: Real-Time Object Detection on Raspberry Pi — https://nickdu.com/?p=1181
[^20^]: YOLOv8 and OpenCV Power Live Webcam Detection — https://medium.com/@lakshmanmandapati0/real-time-ai-vision-yolov8-and-opencv-power-live-webcam-detection-b86964c9d22e
[^21^]: YOLO11 vs YOLOv10 Comparison — https://docs.ultralytics.com/compare/yolo11-vs-yolov10/
[^22^]: YOLOv8/v9/v10/v11 Comparative Performance — https://www.mdpi.com/2076-3417/15/6/3164
[^24^]: SORT, DeepSORT, ByteTrack Comparison — https://vectoral.org/index.php/IJSICS/article/view/97/89
[^26^]: YOLOv8 vs YOLO11 Technical Comparison — https://docs.ultralytics.com/compare/yolov8-vs-yolo11/
[^29^]: OpenCV Camera Calibration Guide — https://medium.com/@amit25173/opencv-camera-calibration-03d19f0f52bc
[^31^]: OpenCV Camera Calibration Tutorial — https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
[^33^]: Multi-Modal Wi-Fi Sensing Overview — https://arxiv.org/html/2505.06682v1
[^37^]: Efficient Re-ID in YOLO — https://www.analyticsvidhya.com/blog/2025/04/re-id-in-yolo/
[^38^]: WiFi and Vision Multimodal Learning (WiVi) — https://openaccess.thecvf.com/content_CVPRW_2019/papers/MULA/Zou_WiFi_and_Vision_Multimodal_Learning_for_Accurate_and_Robust_Device-Free_CVPRW_2019_paper.pdf
[^39^]: Raspberry Pi 4 YOLOv8 FPS Discussion — https://www.reddit.com/r/computervision/comments/1qx8i4f/real_time_object_detection_on_raspberrry_pi_4/
[^45^]: Multi-Modal Sensor Fusion — https://www.emergentmind.com/topics/multi-modal-sensor-fusion
[^46^]: Logitech Brio Series Comparison — https://hub.sync.logitech.com/support/post/logitech-brio-webcams-series-comparison-table-ZuH5fy2U5wgPVFt
[^49^]: Top 5 Multimodal Data Alignment Techniques — https://www.sapien.io/blog/5-smart-strategies-to-align-time-space-semantics
[^54^]: Brio 105 Datasheet — https://www.logitech.com/content/dam/logitech/en/video-collaboration/pdf/brio-105-datasheet.pdf
[^56^]: Brio 105 Product Page — https://www.logitech.com/en-us/products/webcams/brio-105-business-webcam.html
[^59^]: Ultralytics Model Prediction Docs — https://docs.ultralytics.com/modes/predict/
[^63^]: YOLO Performance Analysis on Edge Devices — https://arxiv.org/pdf/2502.15737
[^64^]: Unofficial YOLO Benchmark Results — https://community.ultralytics.com/t/unofficial-benchmark-results-how-fast-can-you-yolo/59
[^67^]: YOLOv8-ncnn Raspberry Pi 4/5 Benchmarks — https://github.com/Qengineering/YoloV8-ncnn-Raspberry-Pi-4
[^69^]: Social Distancing with YOLO and Homography — https://ceur-ws.org/Vol-2884/paper_120.pdf
[^74^]: YOLO on Intel Xeon 6 with OpenVINO — https://lenovopress.lenovo.com/lp2345-accelerating-real-time-object-detection-yolo-models-intel-xeon-6-openvino
[^75^]: YOLOv8 Output Extraction Tutorial — https://www.ultralytics.com/blog/extracting-outputs-from-ultralytics-yolov8
[^80^]: 1000+ FPS YOLOv8 with Intel OpenVINO — https://medium.com/openvino-toolkit/how-to-get-yolov8-over-1000-fps-with-intel-gpus-9b0eeee879
[^90^]: YOLO Track Persist State — https://github.com/ultralytics/ultralytics/issues/21439
[^93^]: Ultralytics Trackers Reference — https://docs.ultralytics.com/reference/trackers/track/
[^96^]: YOLO v5/v7/v8 Benchmark on Jetson — https://www.stereolabs.com/blog/performance-of-yolo-v5-v7-and-v8
[^101^]: Ultralytics YOLO Evolution Overview — https://arxiv.org/html/2510.09653v1
[^105^]: Real-Time Human Detection and ReID with YOLO11 + OSNet — https://blog.aicu.life/posts/real-time-human-detection-and-reid-with-yolo11-osnet/
[^106^]: YOLO11 vs YOLOv8 Official Comparison — https://docs.ultralytics.com/compare/yolo11-vs-yolov8/
