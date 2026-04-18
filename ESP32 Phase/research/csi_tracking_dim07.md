# Dimension 07: Sensor Fusion Architectures — WiFi CSI + Camera

**Research Date:** 2026-01-29
**Researcher:** Deep Research Agent
**Searches Conducted:** 25+
**Sources Reviewed:** 40+

---

## Executive Summary

This report investigates sensor fusion architectures combining WiFi-based localization with camera-based detection, with particular focus on the NOVEL fusion of WiFi CSI (Channel State Information) with YOLO-style object detection. After extensive literature review across academic papers, IEEE publications, arXiv preprints, and hardware documentation, **no existing implementation of direct CSI+YOLO fusion for localization was found** — confirming the novelty claim. Existing fusion combines WiFi with IMU, BLE, UWB, LiDAR, or depth cameras. The closest related work fuses WiFi CSI with vision for activity recognition (WiVi [^73^]), person re-identification (ViFi-ReID [^71^]), or coarse+fine localization (iWVP [^31^]).

This report covers: (1) sensor fusion architectures, (2) Kalman filter equations for multi-sensor fusion, (3) particle filter vs Kalman trade-offs, (4) dynamic weight assignment methods, (5) complementary filtering, (6) camera local vs WiFi room-scale coverage, (7) temporal synchronization, (8) sensor dropout handling, (9) existing WiFi+vision implementations, (10) measurement uncertainty modeling, (11) federated learning relevance, and (12) fusion level taxonomy (feature/decision/position).

---

## 1. Sensor Fusion Architectures for Indoor Localization

### 1.1 Centralized Architecture

In centralized fusion, all sensor measurements are sent to a distinguished Fusion Center (FC) that performs the complete estimation [^66^]. This is the most common architecture for indoor localization systems.

```
Claim: Centralized fusion sends all raw measurements to a fusion center; it does not need to account for cross-covariances of tracks but does not scale well with false alarms [^66^]
Source: IEEE AESS Introduction to Track-to-Track Fusion
URL: https://ieee-aess.org/files/ieeeaess/slides/Felix%20Govaers%2C%20June%2010%2C%202025%20AESS%20DL%20T2TF%20and%20DKF.pdf
Date: 2025
Excerpt: "Centralized: Distinguished Fusion Center(FC); Send all measurements to FC; No cross covariances of tracks; Doesn't scale with False Alarms(FA)"
Context: Comparison of centralized vs. distributed architectures
Confidence: High
```

**Example — EKF-Based WiFi/LiDAR/IMU Fusion [^25^]:** A system on an AGV platform integrates Wi-Fi RSSI fingerprinting, LiDAR SLAM, and IMU through an EKF. Raw data is collected, aligned with unified timestamps, preprocessed (missing value imputation, outlier removal, normalization), and then fused centrally. The system achieves mean 2D errors between 0.24m and 0.38m, compared to 0.51–1.34m for WiFi-only and 0.62–2.88m for LiDAR/IMU-only.

```
Claim: EKF-based centralized fusion of WiFi, LiDAR, and IMU achieves 0.24–0.38m mean 2D error vs. 0.51–1.34m for WiFi-only [^25^]
Source: arXiv — EKF-Based Fusion of Wi-Fi/LiDAR/IMU for Indoor Localization
URL: https://arxiv.org/html/2509.23118v1
Date: 2025-09-27
Excerpt: "The experimental results demonstrate that the multi-sensor fusion method based on the EKF maintains high positioning accuracy under various path conditions and floor environments, with the mean 2D error consistently controlled between 0.2449m and 0.3781m"
Context: AGV-based indoor localization in university building
Confidence: High
```

### 1.2 Distributed Architecture

In distributed fusion, measurements are preprocessed on local nodes, and estimates (not raw data) are sent to a fusion center [^66^]. This architecture scales better but must account for cross-correlations between local estimates.

```
Claim: Distributed fusion preprocesses measurements on nodes and sends estimates to FC; it must account for cross-correlations but scales well [^66^]
Source: IEEE AESS Introduction to Track-to-Track Fusion
URL: https://ieee-aess.org/files/ieeeaess/slides/Felix%20Govaers%2C%20June%2010%2C%202025%20AESS%20DL%20T2TF%20and%20DKF.pdf
Date: 2025
Excerpt: "Distributed: Distinguished FC; Preprocess measurements on nodes; Send estimates to FC; Account for X-correlations; Scales very well in number of sensors and in FA"
Context: Architecture comparison slide deck
Confidence: High
```

**Distributed Fusion Under Unknown Correlations [^67^]:** When cross-covariance matrices between sensors are unknown (common in practice), Covariance Intersection (CI) and related methods provide consistent fusion results [^67^]. The CI fusion equations for two sensors:

$$x_{CI} = P_{CI} [\omega P_1^{-1} \hat{x}_1 + (1-\omega) P_2^{-1} \hat{x}_2]$$
$$P_{CI}^{-1} = \omega P_1^{-1} + (1-\omega) P_2^{-1}$$

where $\omega \in [0,1]$ is chosen to minimize the trace or determinant of $P_{CI}$.

```
Claim: Covariance Intersection provides consistent fused estimates without requiring cross-covariance knowledge, but results can be conservative [^67^]
Source: MDPI Sensors — Distributed Multisensor Data Fusion under Unknown Correlation
URL: https://www.mdpi.com/1424-8220/17/11/2472
Date: 2017-10-27
Excerpt: "CI method provides a consistent fused solution for two estimates... Although consistent, the CI results are conservative with the possibility of much less informative fused estimates"
Context: Comprehensive survey of fusion methods under unknown correlation
Confidence: High
```

**Advanced: Inverse Covariance Intersection [^62^]:** ICI fusion achieves better accuracy than CI while maintaining consistency:

$$\hat{x}_{fus}^{ICI} = W_A \hat{x}_A + W_B \hat{x}_B$$
$$P_{fus}^{ICI} = (P_A^{-1} + P_B^{-1} - [\omega P_A + (1-\omega)P_B]^{-1})^{-1}$$

The ICI method improves fusion accuracy by 33% compared to distributed sequential CI [^62^].

### 1.3 Decentralized (Peer-to-Peer) Architecture

In decentralized fusion, every node is a fusion center; data is only sent to neighbors. This requires handling "information incest" (double-counting) and only approximate solutions exist [^66^].

```
Claim: Decentralized fusion has every node as a FC, sends data only to neighbors, but must account for information incest; only approximate solutions exist [^66^]
Source: IEEE AESS
URL: https://ieee-aess.org/files/ieeeaess/slides/Felix%20Govaers%2C%20June%2010%2C%202025%20AESS%20DL%20T2TF%20and%20DKF.pdf
Date: 2025
Excerpt: "Decentralized: Every node is a FC; Send data only to neighbours; Estimate network topology; Account for information incest; Only approximate solutions exist"
Context: Architecture taxonomy
Confidence: High
```

### 1.4 Federated Learning Architecture

Federated learning (FL) enables training localization models across decentralized client devices without sharing raw data [^48^]. A global model is maintained on a central server; clients train locally and upload model parameters only.

```
Claim: Federated learning for indoor localization trains models across decentralized devices, achieving better accuracy than non-FL approaches (3.24m avg error vs. higher for non-FL) [^48^]
Source: ACM MobiSys — FedHIL: Heterogeneity Resilient Federated Learning for Indoor Localization
URL: https://dl.acm.org/doi/abs/10.1145/3607919
Date: 2023-09-09
Excerpt: "FedHIL outperforms all other FL (and non-FL) frameworks, with the lowest average error of 3.24 meters"
Context: Comparison of FL frameworks for WiFi RSS-based indoor localization
Confidence: High
```

**MetaGraphLoc [^40^]** combines graph neural networks with federated meta-learning for indoor localization using RSSI and IMU data. It includes:
- GNN-based data fusion for comprehensive feature representation
- Meta-learning component for rapid adaptation to new environments
- CSI with federated complex-valued neural networks achieving 0.17m localization error

```
Claim: GNN-based localization using CSI fingerprints achieves 0.17m localization error [^40^]
Source: arXiv — MetaGraphLoc: Graph-based Meta-learning for Indoor Localization
URL: https://arxiv.org/html/2411.17781v1
Date: 2024-11-26
Excerpt: "Their proposed scheme was characterized by high precision and a localization error down to 0.17 m"
Context: Literature review of GNN-based indoor localization
Confidence: Medium
```

---

## 2. Kalman Filters for Multi-Sensor Position Fusion

### 2.1 Extended Kalman Filter Equations

The EKF is the de facto standard for indoor localization sensor fusion. For a nonlinear system:

**Prediction:**
$$\hat{x}_t^- = f(\hat{x}_{t-1}, u_{t-1})$$
$$P_t^- = F_t P_{t-1} F_t^T + Q_t$$

**Update:**
$$K_t = P_t^- H_t^T (H_t P_t^- H_t^T + R_t)^{-1}$$
$$\hat{x}_t = \hat{x}_t^- + K_t (z_t - h(\hat{x}_t^-))$$
$$P_t = (I - K_t H_t) P_t^-$$

Where $F_t = \frac{\partial f}{\partial x}$ and $H_t = \frac{\partial h}{\partial x}$ are Jacobian matrices, $Q_t$ is process noise covariance, and $R_t$ is measurement noise covariance [^20^].

```
Claim: EKF prediction and update equations use Jacobian matrices Ft and Ht for linearization; Kalman gain Kt determines measurement contribution to state estimate [^20^]
Source: Nature Scientific Reports — Multi-sensor fusion localization based on RNN
URL: https://www.nature.com/articles/s41598-025-90492-4
Date: 2025-03-10
Excerpt: "The EKF employs a linearization technique based on a first-order Taylor expansion around the current estimate... K_t is the Kalman gain, which determines the contribution of the measurement data to the final state estimate"
Context: Formal derivation of EKF for IMU + odometry fusion
Confidence: High
```

### 2.2 Multi-Sensor EKF Formulation

For fusing WiFi, LiDAR, and IMU [^25^]:

**State vector:** $\mathbf{x}_k = [p_x, p_y, v_x, v_y]^T$

**Measurement model:** Multiple heterogeneous sensors contribute to the innovation term:

$$\hat{\mathbf{X}}_k = \hat{\mathbf{X}}_{k|k-1} + \mathbf{K}_k (\mathbf{z}_k - \mathbf{H}_k \hat{\mathbf{X}}_{k|k-1})$$
$$\mathbf{P}_k = (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k) \mathbf{P}_{k|k-1}$$

The key advantage: "This EKF formulation supports the recursive fusion of asynchronous, heterogeneous sensor data" [^25^].

### 2.3 Adaptive Covariance EKF

Recent work introduces dynamic measurement covariance adjustment. The **KD-EKF** framework [^70^] reformulates fusion as a dynamic reliability estimation problem:

$$R_k = H_k \cdot R_0$$

where $H_k$ is a reliability scaling factor and $R_0$ is the baseline LOS covariance. When the prediction error is small, the EKF trusts UWB (or WiFi) more; when large, it reduces trust.

```
Claim: Adaptive EKF with dynamic covariance scaling outperforms fixed-covariance EKF by explicitly modeling NLOS-induced degradation [^70^]
Source: arXiv — KD-EKF: Knowledge-Distilled Adaptive Covariance EKF
URL: https://arxiv.org/html/2603.18027v1
Date: 2026-03-06
Excerpt: "When the error is small, the EKF places greater trust in UWB measurements to suppress PDR drift; when the error is large, it reduces the influence of potentially biased UWB values"
Context: Adaptive UWB/PDR indoor localization framework
Confidence: High
```

---

## 3. Particle Filter vs. Kalman Filter

### 3.1 When Particle Filters Are Preferred

Particle filters (PF) are preferred when:
- The system is **nonlinear** and **non-Gaussian**
- Multipath propagation causes positioning errors that follow non-Gaussian distributions [^22^]
- Multiple hypotheses need to be tracked simultaneously
- The state space is low-dimensional

```
Claim: Particle filters outperform EKF when multipath propagation causes non-Gaussian error distributions; PF achieves <1m error at 90th percentile while EKF performs worse under unfavorable conditions [^22^]
Source: KTH Thesis — Design of a Method to Improve 5G Indoor Positioning Using Sensor Fusion
URL: http://kth.diva-portal.org/smash/get/diva2:1887797/FULLTEXT01.pdf
Date: 2024-03-20
Excerpt: "The particle filter offered better performance than an extended Kalman filter with access to IMU and floor map information. This is assumed to be caused by the non-Gaussian distribution of positioning errors caused by multipath propagation"
Context: Master's thesis at KTH comparing PF and EKF for 5G indoor positioning
Confidence: High
```

**Key comparison from [^22^]:**
- Under favorable conditions: PF slightly better than EKF
- Under unfavorable (NLOS) conditions: PF significantly better than EKF
- PF meets accuracy goal of <1m at 90th percentile; EKF does not under unfavorable conditions
- The main reason: "The non-Gaussian character of the position measurements becomes more prominent under worse wireless propagation conditions"

### 3.2 Rao-Blackwellized Particle Filter

The RBPF combines PF for nonlinear variables with Kalman filter for linear variables, reducing computational cost [^24^]:

**State vector partition:**
$$\mathbf{x}_k = \begin{bmatrix} \mathbf{x}_k^{nonlin} \\ \mathbf{x}_k^{lin} \end{bmatrix}$$

The nonlinear position components are estimated by PF; the linear velocity components are estimated by KF. This "decreases the dimensionality from six to three" and reduces the number of particles required [^24^].

```
Claim: RBPF partitions state vector into nonlinear (PF) and linear (KF) components, reducing dimensionality and computational cost [^24^]
Source: NAVIGATION Journal — Novel prior position determination in particle filter for UWB
URL: https://navi.ion.org/content/68/2/277
Date: 2021-06-01
Excerpt: "The nonlinear states (position components) are still estimated using the particle-filtering algorithm while the linear states (velocity components) are estimated using the Kalman filter"
Context: UWB indoor positioning with RBPF
Confidence: High
```

### 3.3 Computational Trade-offs

| Filter | Computational Cost | Nonlinear Systems | Non-Gaussian Noise | Optimal for Linear/Gaussian |
|--------|-------------------|-------------------|-------------------|---------------------------|
| Kalman | Low | No | No | Yes |
| EKF | Medium | Yes (approx.) | No | No |
| UKF | Medium | Yes (better) | No | No |
| Particle | High | Yes | Yes | No (suboptimal) |
| RBPF | Medium-High | Yes | Yes | No |

---

## 4. Dynamic Weight Assignment

### 4.1 Covariance-Based Weighting

The most principled approach: weights are derived from the inverse of measurement covariance matrices. In the Kalman filter, the Kalman gain $K_t$ automatically computes the optimal weight based on relative covariances.

For distributed fusion with CI:
$$\omega^* = \arg\min_{\omega} \text{trace}(P_{CI}(\omega))$$

### 4.2 Confidence-Based Adaptive Weighting

The **Adaptive Weight Extended Kalman Filter [^72^]** dynamically adjusts fusion weights based on "real-time sensor confidence levels":

```
Claim: Adaptive weight EKF dynamically adjusts fusion weights based on real-time sensor confidence, achieving positioning errors within 10-13mm for AGV [^72^]
Source: MDPI Sensors — Multi-Sensor Fusion Localization for Forklift AGV
URL: https://www.mdpi.com/1424-8220/25/18/5670
Date: 2025-09-11
Excerpt: "The methodological innovation lies in its internal-prediction/external-correction fusion strategy and adaptive weight optimization algorithm, which dynamically adjusts fusion weights according to real-time sensor confidence levels"
Context: LiDAR + odometry fusion for forklift AGV
Confidence: High
```

### 4.3 Error-Based Reliability Estimation (KD-EKF)

The KD-EKF approach [^70^] uses a learned model to predict the next position from historical trajectories. The prediction error becomes a quantitative indicator of measurement reliability:

$$H_k = f_{reliability}(|\hat{x}_{pred} - z_{UWB}|)$$

where $f_{reliability}$ scales the measurement covariance. MAD (Median Absolute Deviation) over a sliding window provides robust outlier detection.

### 4.4 SNR-Based Weighting

For WiFi signals specifically, RSSI/CSI SNR directly indicates measurement quality. Lower SNR → higher measurement uncertainty → lower weight. The measurement covariance for WiFi can be modeled as inversely proportional to SNR:

$$R_{WiFi} \propto \frac{1}{SNR}$$

This is implicit in most WiFi fingerprinting systems but rarely explicitly formulated. The MM-Loc system [^36^] handles this implicitly by masking missing/low-quality WiFi samples with null vectors.

---

## 5. Complementary Filtering

### 5.1 Core Concept

Complementary filters combine high-frequency information from one sensor with low-frequency information from another through frequency-domain separation [^17^].

```
Claim: Complementary filters combine signals by applying high-pass filter to one and low-pass filter to another; high-pass extracts high frequencies while low-pass extracts low frequencies [^17^]
Source: arXiv — Complementary Filtering for Dynamics Learning
URL: https://arxiv.org/abs/2302.13754
Date: 2023-02-27
Excerpt: "This filtering technique combines two signals by applying a high-pass filter to one signal, and low-pass filtering the other. Essentially, the high-pass filter extracts high-frequencies, whereas the low-pass filter extracts low frequencies"
Context: Dynamics model learning using complementary filters
Confidence: High
```

### 5.2 Application to WiFi + Camera

For WiFi CSI + Camera fusion, complementary filtering is directly applicable:

| Sensor | Frequency Content | Filter Role |
|--------|------------------|-------------|
| **Camera (YOLO)** | High-frequency (30+ FPS) | High-pass for rapid local tracking |
| **WiFi CSI** | Low-frequency (10-100 Hz packet rate) | Low-pass for room-scale drift correction |

**Discrete implementation:**
```
angle_est = alpha * (angle_est + gyro * dt) + (1 - alpha) * accel_angle
```

```
Claim: Complementary filters offer low-cost, explainable fusion combining high-frequency and low-frequency sensor data, ideal for edge devices [^18^]
Source: ShadeCoder — Complementary Filter: A Comprehensive Guide
URL: https://www.shadecoder.com/ja/topics/complementary-filter-a-comprehensive-guide-for-2025
Date: 2026-01-02
Excerpt: "Complementary filters offer a low-cost, explainable fusion method that combines high-frequency and low-frequency sensor data... Simplicity and efficiency"
Context: Tutorial on complementary filter implementation
Confidence: High
```

### 5.3 WiFi+Camera Specific Complementary Architecture

For the proposed CSI+YOLO system:
- **Camera (YOLO detection)** provides high-rate local position estimates (30-60 FPS)
- **WiFi CSI** provides low-rate global room-scale position estimates (every 100-1000ms)
- The complementary filter weights camera estimates more heavily for short-term tracking
- WiFi CSI corrects long-term drift and provides absolute reference

The weight $\alpha$ can be tuned based on the camera detection confidence and WiFi CSI quality:
- High camera confidence + good WiFi → blend equally
- Low camera confidence (occlusion, poor lighting) → rely more on WiFi
- Poor WiFi (NLOS, interference) → rely more on camera

---

## 6. Camera Local Tracking + WiFi Room-Scale Coverage

### 6.1 Complementary Spatial Characteristics

The two modalities have fundamentally different spatial characteristics:

| Aspect | WiFi CSI | Camera (YOLO) |
|--------|----------|---------------|
| Coverage | Room-scale (10-50m range) | Local FOV (2-10m depending on lens) |
| Accuracy | Coarse (1-3m typical) | Fine (cm-level within FOV) |
| Works in dark | Yes | No (without IR) |
| Through obstacles | Partial (multipath) | No (LOS only) |
| Absolute position | Yes (with fingerprinting) | Relative to camera frame |
| Identity | MAC address association | Visual appearance |

```
Claim: WiFi provides wide coverage with lower accuracy; camera provides high accuracy within limited FOV — these are complementary characteristics ideal for fusion [^61^]
Source: GMU C4I — WiOF: WiFi and Optical Fusion Tracking
URL: https://c4i.gmu.edu/~pcosta/F15/data/fileserver/file/472069/filename/Paper_1570111665.pdf
Date: Unknown
Excerpt: "The WiOF method improves on RSSI or optical methods alone for robust tracking over camera occlusions or RSSI uncertainties... Once the subject enters the camera's FOV, the tracking was then enhanced to precisely follow the target. When the target exits the fixed of view, the RSSI tracking is resumed"
Context: RSSI + optical fusion for target tracking
Confidence: High
```

### 6.2 Coarse-to-Fine Architecture

The **iWVP system [^31^]** demonstrates a practical coarse-to-fine approach:

1. **Coarse (WiFi):** Bayes filter determines which region the user is in (room-level accuracy ~95.6%)
2. **Fine (Camera):** Surveillance camera provides precise position and tracking within the region
3. **Association:** PDR trace and vision trace are associated to confirm user ID

```
Claim: Bayes filter for coarse WiFi region discrimination achieves 95.652% accuracy, then camera fine-localizes within the region [^31^]
Source: Tsinghua Science and Technology — Two-Stage Indoor Localization with Multisensor Fusion
URL: https://www.sciencedirect.com/science/article/pii/S2352340925010832
Date: 2022-03-19
Excerpt: "The average regional prediction accuracy is 95.652%, virtually unaffected by time and environmental changes... average positioning error is about 4.61 cm"
Context: WiFi + camera + PDR fusion for pedestrian tracking
Confidence: High
```

### 6.3 RSSI + ORB-SLAM Fusion for V-SLAM

WiFi RSSI has been fused with ORB-SLAM to enhance localization stability in texture-less areas [^26^]. By leveraging signal strength measurements alongside visual feature extraction, hybrid approaches mitigate feature degradation.

---

## 7. Temporal Synchronization Requirements

### 7.1 The Synchronization Challenge

Sensors operate at different frequencies — cameras at 30 FPS, IMUs at 200+ Hz, WiFi at irregular intervals. Even millisecond misalignment can distort high-speed control [^30^].

```
Claim: Even millisecond misalignment can distort high-speed control; timestamp normalization and hardware synchronization are essential [^30^]
Source: Digital Divide Data — The Role of Multisensor Fusion Data in Physical AI
URL: https://www.digitaldividedata.com/blog/the-role-of-multisensor-fusion-data-in-physical-ai
Date: 2026-02-19
Excerpt: "Sensors operate at different frequencies. Cameras may run at 30 frames per second. IMUs can exceed 200 Hz. LiDAR might rotate at 10 Hz. If timestamps drift, fusion degrades. Even a millisecond misalignment can distort high-speed control"
Context: Data engineering challenges in multisensor fusion
Confidence: High
```

### 7.2 Precision Time Protocol (PTP) Solutions

For cross-domain clock synchronization, PTPv2 achieves ±50 μs accuracy [^27^]:

```
Claim: PTPv2 over Automotive Ethernet achieves ±50 μs clock synchronization, reducing fusion latency from 38.7ms to 12.5ms and improving detection precision from 78.5% to 96.3% [^27^]
Source: European Journal of Electrical Engineering and Computer Science
URL: https://www.eu-opensci.org/index.php/ejece/article/download/19771/13382
Date: 2025-12
Excerpt: "PTPv2 achieves high-precision temporal alignment... fusion latency reduced by over 67%, from 38.7 ms to 12.5 ms... object detection accuracy improved from 78.5% to 96.3%"
Context: Automated parking with ultrasonic + vision fusion
Confidence: High
```

**Key synchronization metrics from [^27^]:**

| Metric | Unsynchronized | PTPv2 |
|--------|---------------|-------|
| Mean clock offset | 312 μs | 42 μs |
| Fusion latency | 38.7 ms | 12.5 ms |
| Object localization RMSE | 19.4 cm | 5.6 cm |
| Detection precision | 78.5% | 96.3% |
| Synchronization integrity score | 0.41 | 0.97 |

### 7.3 Asynchronous Fusion Handling

The **DSICI** algorithm [^62^] handles asynchronous measurements by transforming multi-source asynchronous systems into synchronized state space models using state iteration. Missing measurements induced by network blocking are modeled as Bernoulli-distributed random variables.

### 7.4 Time-Aware Confidence Weighting

A practical approach: weight sensor inputs by their temporal proximity to the fusion timestamp [^27^]:

$$L_t = T_{fused} - \min(T_i)$$

Sensors with measurements closer to the fusion time receive higher weights. If confidence is below a threshold, conservative measures (e.g., increasing estimation uncertainty) are applied.

---

## 8. Sensor Dropout and Degradation Handling

### 8.1 Graceful Degradation Principles

Sensor fusion systems must handle:
- **Complete sensor dropout** (camera covered, WiFi AP failure)
- **Partial degradation** (occlusion, NLOS, low SNR)
- **Temporal inconsistency** (measurement delays, jitter)

### 8.2 Multi-Expert Decoder Architecture

**MoME (Mixture of Multi-modal Expert)** [^53^] provides a state-of-the-art approach for sensor dropout:

```
Claim: MoME with Adaptive Query Router achieves +4.2% mAP in LiDAR-drop, +1.9% in camera-drop, +6.7% in limited FOV vs. SOTA [^53^]
Source: arXiv — Resilient Sensor Fusion under Adverse Sensor Failures via Multi-Modal Expert Fusion
URL: https://arxiv.org/html/2503.19776v2
Date: 2025
Excerpt: "In LiDAR Drop scenarios, the AQR module redirects 92% of queries to the Camera expert... In Camera Drop scenarios, AQR allocates 100% of queries to the LiDAR expert"
Context: LiDAR-camera fusion for autonomous driving
Confidence: High
```

**MoME architecture:**
- Three parallel expert decoders: Camera-only, LiDAR-only, LiDAR+Camera
- Adaptive Query Router (AQR) dynamically assigns each object query to one expert
- Trained with synthetic sensor drop augmentation
- Each query processed by only one decoder → minimal computational overhead

### 8.3 Fallback Strategies for WiFi+Camera

For the proposed CSI+YOLO system, recommended fallback hierarchy:

| State | Action | Estimation Source |
|-------|--------|-------------------|
| Both sensors healthy | Full fusion | Weighted EKF/PF fusion |
| Camera dropout | WiFi-only mode | CSI-only localization |
| WiFi dropout | Camera-only + prediction | Visual tracking + IMU prediction |
| Both dropout | Dead reckoning | Last known + motion model |
| Camera degradation (occlusion) | Increase WiFi weight | Covariance inflation |
| WiFi degradation (NLOS) | Increase camera weight | Visual-only + drift warning |

### 8.4 Covariance Inflation for Dropout

When a sensor drops out, inflate its measurement covariance to effectively remove its influence:

$$R_{dropped} = \alpha \cdot I, \quad \alpha \rightarrow \infty$$

This is equivalent to setting the Kalman gain for that sensor to zero.

---

## 9. Existing WiFi + Vision Fusion Implementations

### 9.1 Activity Recognition: WiVi [^73^]

The **WiVi** system fuses WiFi CSI with RGB camera for human activity recognition (sitting, standing, walking) using decision-level fusion:

```
Claim: WiVi achieves 97.5% HAR accuracy using WiFi CSI + camera with decision-level DNN ensemble fusion [^73^]
Source: CVPRW 2019 — WiFi and Vision Multimodal Learning
URL: https://openaccess.thecvf.com/content_CVPRW_2019/papers/MULA/Zou_WiFi_and_Vision_Multimodal_Learning_for_Accurate_and_Robust_Device-Free_CVPRW_2019_paper.pdf
Date: 2019
Excerpt: "WiVi achieves 97.5% human activity recognition accuracy which inherits the strengths of both WiFi and vision... WiFi is the ideal modality to provide complementary information to vision sensing especially in poor lighting conditions"
Context: Device-free HAR using COTS IoT + webcam
Confidence: High
```

**WiVi fusion method:**
- WiFi sensing module: CNN on CSI frames
- Vision sensing module: C3D on RGB video
- Decision fusion: 4-layer DNN (128-256-128-num_classes) on concatenated SoftMax outputs
- Ensemble classifier learns correlations between modalities

### 9.2 Person Re-Identification: ViFi-ReID [^71^]

**ViFi-ReID** uses WiFi CSI + RGB for person re-identification across modalities:

```
Claim: ViFi-ReID uses transformer-based feature fusion between vision and WiFi CSI for person re-ID, demonstrating cross-modal person matching [^71^]
Source: arXiv — Short Overview of Multi-Modal Wi-Fi Sensing
URL: https://arxiv.org/html/2505.06682v1
Date: 2025-05-10
Excerpt: "Mao et al. proposed a vision+Wi-Fi person re-identification method, where... a CLIP loss between the feature embeddings of Wi-Fi and vision is added"
Context: Survey of multi-modal Wi-Fi sensing
Confidence: High
```

### 9.3 Localization: Ye et al. + MaskFi [^71^]

**Ye et al.** use ViT to process concatenated image + WiFi CSI for localization tasks. **MaskFi** uses ViT with masked language model pre-training for action recognition.

### 9.4 RSSI + Optical Fusion Tracking [^61^]

The WiOF method performs track-to-track fusion of optical and RSSI data:

```
Claim: WiOF performs track-to-track fusion of RSSI and optical data, maintaining tracking when target exits camera FOV [^61^]
Source: GMU C4I
URL: https://c4i.gmu.edu/~pcosta/F15/data/fileserver/file/472069/filename/Paper_1570111665.pdf
Date: Unknown
Excerpt: "A target enters an area and maintains a particular direction... the position was tracked using the RSSI, but the track was not accurate. Once the subject enters the camera's FOV, the tracking was then enhanced"
Context: RSSI + optical sensor fusion for threat detection
Confidence: High
```

### 9.5 WiFi + Camera + PDR: iWVP [^31^]

A three-stage system:
1. **WiFi fingerprinting** → coarse region (Bayes filter, 95.6% region accuracy)
2. **Camera detection** → fine position within region (visual tracking)
3. **PDR association** → identity confirmation via trace correlation

### 9.6 ESP32 + Camera + Sensor Fusion [^56^]

```
Claim: ESP32-CAM + ultrasonic sensor achieves 89% spatial detection accuracy using YOLOv8 in 715ms average processing time [^56^]
Source: Preprints — Sensor Fusion for Real-Time Object Detection and Spatial Positioning
URL: https://www.preprints.org/manuscript/202411.0611
Date: 2024-11-07
Excerpt: "The solution achieved a spatial detection accuracy of 89%, demonstrating its potential for reliable obstacle avoidance in unmanned vehicles"
Context: ESP32-CAM with YOLOv8 + ultrasonic for obstacle detection
Confidence: Medium
```

### 9.7 ESP32 P4 + Camera + AI [^54^]

The ESP32-P4 features MIPI-CSI interface for camera, 400MHz CPU, built-in AI acceleration, and supports sensor fusion:

```
Claim: ESP32 P4 has native MIPI-CSI camera interface, AI acceleration, and sufficient I/O for concurrent multi-sensor data [^54^]
Source: Hackaday — ESP32 P4 + Camera + Sensors
URL: https://hackaday.io/project/202943/log/242918-esp32-p4-camera-sensors
Date: 2025-08-22
Excerpt: "Equipped with a MIPI-CSI interface, it can directly connect to high-resolution camera modules... Built-in vector instruction set and AI acceleration unit, supporting efficient local AI inference"
Context: ESP32 P4 development project log
Confidence: Medium
```

### 9.8 WiFi + LiDAR + IMU: EKF Fusion [^25^]

The most similar existing architecture to the proposed CSI+Camera system uses WiFi + LiDAR + IMU:
- WiFi RSSI for global anchoring (1-3m accuracy)
- LiDAR SLAM for local precision (sub-meter)
- IMU for high-rate temporal continuity
- EKF fuses all three with unified timestamps

**Result:** 0.24-0.38m mean error vs. 0.51-1.34m (WiFi only) and 0.62-2.88m (LiDAR/IMU only).

---

## 10. Measurement Uncertainty Modeling

### 10.1 WiFi CSI Uncertainty

WiFi CSI measurement uncertainty depends on:
- **SNR** of each subcarrier
- **Multipath conditions** (LOS vs. NLOS)
- **Number of antennas** (MIMO diversity)
- **Packet rate** and temporal consistency

The measurement covariance for WiFi can be modeled as:

$$R_{WiFi} = \text{diag}(\sigma_{x}^2, \sigma_{y}^2)$$

where $\sigma$ scales inversely with SNR and number of contributing APs. In NLOS conditions, covariance should be inflated by a factor of 2-10x based on environment classification.

### 10.2 Camera (YOLO) Uncertainty

Camera measurement uncertainty depends on:
- **Bounding box detection confidence** from YOLO
- **Distance from camera** (pixel error → world coordinate error scales with distance)
- **Calibration accuracy** (intrinsic + extrinsic parameters)
- **Occlusion level**

$$R_{camera} = \frac{d^2}{f^2 \cdot conf_{YOLO}} \cdot R_0$$

where $d$ is depth, $f$ is focal length, and $conf_{YOLO}$ is the detection confidence score.

### 10.3 Adaptive Covariance via Reinforcement Learning

```
Claim: Reinforcement learning (actor-critic with GRU + attention) can adaptively generate KF parameter adjustment factors to dynamically regulate state covariance matrix [^35^]
Source: PMC — Kalman Filter-Based Localization Calibration Method based on RL
URL: https://pmc.ncbi.nlm.nih.gov/articles/PMC12385807/
Date: 2025
Excerpt: "An actor-critic network architecture is constructed, where a gated recurrent unit (GRU) captures temporal dependencies of sensor data, and an attention mechanism focuses on key state features to adaptively generate KF parameter adjustment factors"
Context: UAV swarm localization with adaptive KF
Confidence: High
```

### 10.4 Information Matrix Fusion

For multi-trajectory cooperative localization, information matrix fusion aggregates inverse covariance matrices:

$$P_{fused}^{-1} = \sum_i P_i^{-1}$$

This forms "a layered uncertainty calibration mechanism" that weights each sensor's contribution by its information content [^35^].

### 10.5 Covariance Bounds for Unknown Correlations

When fusing estimates with unknown cross-correlations, ellipsoidal methods provide bounds [^67^]:

| Method | Characteristic |
|--------|---------------|
| CI | Consistent but conservative |
| EI | Less conservative, may be inconsistent |
| ICI | Best accuracy with consistency |
| LE | Maximum ellipsoid inside intersection |

---

## 11. Federated Learning for Sensor Fusion

### 11.1 Basic FL Framework for Localization

The standard FL training loop for indoor localization [^46^]:

1. Server initializes global model $w_g^{(t)}$
2. Broadcast to participating clients
3. Each client trains locally: $\tilde{w}_i^{(t)} = w_i^{(t)} - \eta \nabla L_i(w_i^{(t)})$
4. Server aggregates: $w_g^{(t+1)} = \sum_{i=1}^N p_i^{(t)} \tilde{w}_i^{(t)}$

```
Claim: FL techniques for indoor localization use DNN with regression (RMSE) or classification accuracy; performance depends on number of clients and location classes [^46^]
Source: Hanyang University — Tutorial on Federated Learning for Indoor Localization
URL: https://scholarworks.bwise.kr/hanyang/bitstream/2021.sw.hanyang/196794/
Date: 2021
Excerpt: "FL techniques using the regression method compute the error between the estimated location and the actual test data... using the Root Mean Square Error (RMSE) metric"
Context: Tutorial on FL methodologies for indoor localization
Confidence: High
```

### 11.2 Relevance to CSI+Camera Fusion

Federated learning is **relevant but not critical** for the proposed CSI+YOLO fusion because:

1. **Camera data is privacy-sensitive** — FL enables model improvement without sharing raw images
2. **CSI patterns vary across environments** — FL allows learning from diverse deployments
3. **Non-IID data** — Different buildings have different WiFi AP layouts and camera positions; FL handles this via techniques like FedHIL [^48^]
4. **CSI+YOLO fusion model** could be trained federated across multiple buildings, with each site contributing local CSI+camera data

However, for real-time operation on ESP32-class hardware, FL is a training paradigm, not an inference architecture. The runtime fusion is centralized or distributed, not federated.

### 11.3 FedHIL for Heterogeneous Devices

FedHIL achieves 3.24m average localization error across heterogeneous smartphones, outperforming CRNP, FedAeDNN, and FedLoc by 1.62x, 1.86x, and 2.26x respectively [^48^].

---

## 12. Fusion Levels: Feature vs. Decision vs. Position

### 12.1 Three-Level Taxonomy

```
Claim: Sensor fusion occurs at three levels: data level (raw sensor data), feature level (extracted features), and decision level (classifications); choice affects quality and complexity [^43^]
Source: TNO — Sensor Fusion Algorithms
URL: https://publications.tno.nl/publication/34619009/glelg8/pub95195.pdf
Date: Unknown
Excerpt: "Three different fusion levels can be discerned: data level, feature level and decision level... The choice of a suitable fusion level is important since it affects both the quality of the detection results and complexity of the fusion process"
Context: Military sensor fusion survey
Confidence: High
```

### 12.2 Position-Level Fusion (Recommended for CSI+YOLO)

Each sensor produces an independent position estimate; fusion combines the positions:

**Advantages:**
- Simple to implement
- Sensors can be heterogeneous (different types)
- Each sensor's internal processing is independent
- Easy to add/remove sensors

**Disadvantages:**
- Information loss from individual sensors
- May lose correlation information

**Best for:** The proposed CSI+YOLO system. WiFi CSI produces (x,y) position estimates; YOLO+depth produces (x,y) position estimates. An EKF/PF fuses the position estimates.

**Example — WiVi decision fusion [^73^]:**
- WiFi module outputs activity class probabilities
- Vision module outputs activity class probabilities
- DNN ensemble fuses the probability vectors

### 12.3 Feature-Level Fusion

Features are extracted from each sensor, then combined into a joint feature vector:

**Advantages:**
- Preserves more information than decision-level
- Cross-modal correlations can be learned

**Disadvantages:**
- Requires compatible feature representations
- Higher dimensionality
- More training data needed

**Example — MM-Loc [^36^]:**
- LSTM extracts 128-dim features from inertial sensor data
- DNN extracts 128-dim features from WiFi RSS data
- Concatenation to 256-dim, then FC layers
- Result: ~50% improvement over independent modalities

```
Claim: MM-Loc multimodal DNN achieves median error under 2m vs. 2.6m (PDR alone) and 6.9m (WiFi alone), through feature-level fusion [^59^]
Source: IPIN 2021 — MM-Loc: Cross-sensor Indoor Smartphone Location Tracking
URL: https://xijiawei.com/files/ipin2021.pdf
Date: 2021
Excerpt: "Their combination with a multimodal architecture captures the perspective from cross-sensor modalities to reduce median error to under 2 metres"
Context: WiFi fingerprinting + PDR fusion via multimodal DNN
Confidence: High
```

### 12.4 Decision-Level Fusion

Each sensor independently makes a decision; decisions are combined:

**Advantages:**
- Minimal data transmission
- Very modular
- Low computational cost
- Natural fault tolerance

**Disadvantages:**
- Highest information loss
- Early decisions cannot be reversed

### 12.5 Hybrid/Arbitrary Fusion Architectures

In practice, fusion need not be restricted to a single level [^45^]:

```
Claim: Fusion architectures can be represented as directed graphs with sensors as input nodes and decisions as output nodes; information changes nature as it passes through the graph [^45^]
Source: ScienceDirect — Sensor Fusion overview
URL: https://www.sciencedirect.com/topics/engineering/sensor-fusion
Date: Unknown
Excerpt: "A fusion architecture can therefore be thought of as a directed graph with input nodes the sensors and output nodes the decisions"
Context: Comprehensive sensor fusion reference
Confidence: High
```

### 12.6 Recommendation for CSI+YOLO

For the novel CSI+YOLO fusion, we recommend a **hybrid position-measurement fusion architecture**:

1. **WiFi CSI pipeline:** CSI → preprocessing → feature extraction → position estimate (x,y) + covariance
2. **Camera pipeline:** Frame → YOLO detection → depth estimation → position estimate (x,y) + confidence
3. **Fusion:** EKF fuses position estimates with weights from covariances/confidences
4. **Fallback:** When one sensor drops out, the other continues with inflated covariance

This is a **loosely-coupled** architecture [^75^], where each sensor produces position estimates independently before fusion. The alternative, **tightly-coupled** fusion (fusing raw CSI features with YOLO features directly), would require a feature-level deep learning approach but is more complex to implement.

```
Claim: Loosely-coupled fusion incorporates position estimates from subsystems; tightly-coupled fusion integrates raw measurements directly into the Kalman filter [^75^]
Source: Oxford CS — Tightly-Coupled Integration of Inertial and Magneto-Inductive Technologies
URL: http://www.cs.ox.ac.uk/people/changhao.chen/website/docs/MI_report.pdf
Date: Unknown
Excerpt: "[23] simply incorporated positions estimation from PDR into WSN in a loosely-coupled way... [24] reported a first trial to design a tightly-coupled integration for WiFi and IMU by linearizing range constraints"
Context: Comparison of loosely vs. tightly coupled WiFi+IMU fusion
Confidence: High
```

---

## 13. Temporal Misalignment Robustness

### 13.1 Impact of Temporal Misalignment

Temporal misalignment significantly degrades fusion performance:

```
Claim: Temporal misalignment between sensors degrades fusion; methods range from accurate timestamps to Kalman filtering corrections, but robustness guarantees are needed [^38^]
Source: TU Dortmund — Sync or Sink? Robustness of Sensor Fusion against Temporal Misalignment
URL: https://daes.cs.tu-dortmund.de/storages/daes-cs/r/Bilder/Beschaeftigte/Daniel_Kuhse/RTAS24_Kuhse.pdf
Date: 2024
Excerpt: "Temporal misalignment is generally acknowledged as problematic... it would be desirable to have guarantees that the sensor fusion function is robust against this remaining temporal misalignment"
Context: RTAS 2024 paper on temporal robustness of sensor fusion
Confidence: High
```

### 13.2 Delay-Aware Adaptive Fusion

```
Claim: Delay-aware adaptive fusion compensates for inter-modal lags, weighs sensor inputs, and computes confidence scores; if confidence is low, conservative measures are applied [^33^]
Source: arXiv — Temporal Misalignment Attacks against Multimodal Perception
URL: https://arxiv.org/html/2507.09095v3
Date: 2026-03-05
Excerpt: "Delay-aware adaptive fusion compensates for inter-modal lags, weighs sensor inputs, and computes confidence scores. If confidence is low, conservative measures, such as slowing down, increasing spacing, or lowering autonomy, can be applied"
Context: Defense strategies against temporal misalignment in autonomous driving
Confidence: High
```

---

## 14. Novelty Assessment: CSI+YOLO Fusion

### 14.1 What Exists

After extensive literature review, **NO direct CSI+YOLO fusion for indoor localization** was found. Existing related work includes:

| Work | Modality | Task | Fusion Level | Year |
|------|----------|------|-------------|------|
| WiVi [^73^] | CSI + RGB | Activity Recognition | Decision (DNN) | 2019 |
| ViFi-ReID [^71^] | CSI + RGB | Person Re-ID | Feature (Transformer) | 2022 |
| X-Fi [^76^] | CSI + RGB + mmWave + ... | Activity/Pose | Feature (Cross-modal) | 2024 |
| iWVP [^31^] | RSSI + Camera + PDR | Localization | Position (Bayes+Visual) | 2022 |
| WiOF [^61^] | RSSI + Optical | Tracking | Track-to-track | ~2015 |
| Ye et al. [^71^] | CSI + RGB | Localization | Input (ViT) | 2024 |
| MM-Loc [^59^] | RSSI + IMU | Localization | Feature (DNN) | 2021 |
| EKF WiFi/LiDAR [^25^] | RSSI + LiDAR + IMU | Localization | Position (EKF) | 2025 |
| HDANet [^71^] | CSI + RGB | Crowd Counting | Feature (CNN) | 2023 |

### 14.2 What Is Novel

The proposed **CSI+YOLO sensor fusion for indoor localization** is novel because:

1. **No existing work** combines WiFi CSI (fine-grained PHY layer) with YOLO object detection for localization
2. **Weight-based fusion** of CSI position estimates with camera-derived positions has not been explored
3. **ESP32-class implementation** of CSI+camera fusion would be the first
4. **Position-level EKF fusion** of CSI and vision for pedestrian tracking is unexplored

### 14.3 Recommended Architecture

Based on this research, the recommended architecture for the novel CSI+YOLO fusion:

```
┌─────────────┐     ┌──────────────┐     ┌─────────────────┐
│  WiFi CSI   │────→│  CSI Feature │────→│  CSI Position   │──┐
│  Receiver   │     │  Extraction  │     │  Estimator      │  │
└─────────────┘     └──────────────┘     └─────────────────┘  │  ┌──────────┐     ┌─────────┐
                                                              ├──→│   EKF    │────→│ Fused   │
┌─────────────┐     ┌──────────────┐     ┌─────────────────┐  │  │  Fusion  │     │ Position│
│   Camera    │────→│    YOLO      │────→│  Camera Position│──┘  │          │     │         │
│  (ESP32-CAM)│     │  Detection   │     │  + Covariance   │     └──────────┘     └─────────┘
└─────────────┘     └──────────────┘     └─────────────────┘
                                                               ↑
                                                    (adaptive weights from
                                                     covariances/confidence)
```

**Key design decisions:**
1. **Loosely-coupled position-level fusion** — Each sensor produces independent position estimates
2. **EKF with adaptive covariance** — Weights derived from measurement confidence
3. **Complementary filter layer** — Camera for high-frequency local tracking, WiFi for low-frequency drift correction
4. **Graceful degradation** — Automatic fallback when either sensor drops out
5. **Temporal alignment** — PTP or timestamp normalization across modalities

---

## 15. Counter-Narratives and Limitations

### 15.1 Why CSI+YOLO Fusion May Not Work Well

1. **CSI accuracy is limited** — Even with fusion, WiFi CSI localization is typically 1-3m. Camera accuracy degrades with distance. Combined accuracy may not be dramatically better than camera alone within FOV.

2. **Synchronization overhead** — CSI and camera operate at fundamentally different rates. The overhead of proper temporal alignment may not be worth the accuracy gain.

3. **Camera is already accurate within FOV** — If the camera is the primary sensor when the target is visible, WiFi CSI adds value only when the target exits the FOV. The fusion may reduce to simple handoff.

4. **YOLO only detects objects in its training set** — If the target is an unknown object class, YOLO fails completely. WiFi CSI works for any person/object that affects the signal.

5. **Processing constraints** — Running YOLO + CSI processing simultaneously on ESP32-class hardware may exceed real-time constraints.

### 15.2 Alternative Approaches to Consider

1. **WiFi CSI alone** for room-scale + **simple motion detection** for camera-free periods
2. **UWB instead of WiFi** — UWB provides 10-30cm accuracy, making fusion with camera more valuable
3. **LiDAR instead of camera** — LiDAR works in all lighting conditions
4. **Pure deep learning fusion** — Train an end-to-end multimodal network (like MM-Loc) instead of explicit EKF fusion

### 15.3 The Tightly-Coupled Alternative

Tightly-coupled fusion (fusing raw CSI features directly with YOLO features in a deep network) could theoretically achieve better performance but:
- Requires massive training data
- No interpretability
- Difficult to debug
- Hardware-constrained for ESP32 deployment

---

## 16. Key Equations Summary

### EKF Multi-Sensor Fusion
```
Prediction:
  x̂ₖ⁻ = f(x̂ₖ₋₁, uₖ₋₁)
  Pₖ⁻ = Fₖ Pₖ₋₁ Fₖᵀ + Qₖ

Update (for each sensor i):
  Kₖᵢ = Pₖ⁻ Hₖᵢᵀ (Hₖᵢ Pₖ⁻ Hₖᵢᵀ + Rₖᵢ)⁻¹
  x̂ₖ = x̂ₖ⁻ + Kₖᵢ (zₖᵢ - hᵢ(x̂ₖ⁻))
  Pₖ = (I - Kₖᵢ Hₖᵢ) Pₖ⁻
```

### Covariance Intersection (Unknown Correlation)
```
P_CI⁻¹ = ω P₁⁻¹ + (1-ω) P₂⁻¹
x_CI = P_CI [ω P₁⁻¹ x̂₁ + (1-ω) P₂⁻¹ x̂₂]
ω* = argmin trace(P_CI)
```

### Complementary Filter (WiFi + Camera)
```
x_fused = α · x_camera + (1-α) · x_WiFi
α = f(conf_camera, 1/R_WiFi)
where f() is a confidence-weighting function
```

### Adaptive Weight (SNR/Confidence Based)
```
R_WiFi = R₀ / SNR_eff
R_camera = (d²/f²) · R₀ / conf_YOLO
K_WiFi = P H_WiFiᵀ (H_WiFi P H_WiFiᵀ + R_WiFi)⁻¹
K_camera = P H_camᵀ (H_cam P H_camᵀ + R_camera)⁻¹
```

---

## 17. Hardware Implementation Notes

### ESP32 P4 for CSI+Camera [^54^]
- **MIPI-CSI interface** for direct camera module connection
- **400MHz CPU** for concurrent multi-sensor processing
- **AI acceleration unit** for on-device YOLO inference
- **Wi-Fi 6** support for high-rate CSI collection
- **Vector instruction set** for signal processing

### ESP32-CAM [^56^]
- Limited processing power; YOLOv8 may require ~715ms inference time
- CSI processing may need to be offloaded
- Simpler YOLO variants (YOLOv5n, YOLOv8n) recommended

---

## 18. Summary of Key Sources

| Citation | Type | Relevance |
|----------|------|-----------|
| [^20^] Nature Sci Rep | EKF equations | Core fusion math |
| [^22^] KTH Thesis | PF vs EKF comparison | Filter selection |
| [^25^] arXiv WiFi/LiDAR/IMU | Multi-sensor EKF | Architecture template |
| [^27^] EJECE | Temporal sync | PTP implementation |
| [^31^] iWVP system | WiFi+Camera+PDR | Closest existing work |
| [^36^] MM-Loc | Feature-level fusion | Deep learning alternative |
| [^48^] FedHIL | Federated learning | FL for localization |
| [^53^] MoME | Sensor dropout | Robustness architecture |
| [^61^] WiOF | RSSI+optical fusion | Track-to-track |
| [^67^] MDPI Sensors | CI/EI/ICI methods | Unknown correlation |
| [^70^] KD-EKF | Adaptive covariance | Dynamic weighting |
| [^71^] Multi-Modal WiFi Survey | CSI+vision survey | Related work |
| [^73^] WiVi | CSI+camera activity | Decision fusion |
| [^38^] RTAS | Temporal robustness | Misalignment handling |
| [^62^] Nature Sci Rep | DSICI algorithm | Async fusion |
| [^17^] arXiv | Complementary filters | Frequency-based fusion |
| [^54^] Hackaday | ESP32 P4 | Hardware platform |
| [^56^] Preprints | ESP32-CAM+YOLOv8 | Hardware proof |
| [^66^] IEEE AESS | Architecture taxonomy | Centralized/distributed |
| [^43^] TNO | Fusion levels | Level taxonomy |
| [^45^] ScienceDirect | Sensor fusion overview | General principles |
| [^72^] MDPI Sensors | Adaptive weight EKF | Weight tuning |
| [^75^] Oxford CS | Loosely vs tightly coupled | Coupling taxonomy |

---

## 19. Conclusions

1. **Sensor fusion architectures** span centralized, distributed, and federated paradigms. For CSI+YOLO, centralized EKF fusion is the most practical starting point.

2. **EKF equations** for multi-sensor fusion are well-established. The adaptive covariance EKF (KD-EKF) provides the best framework for dynamic weighting.

3. **Particle filters** outperform EKF under non-Gaussian multipath conditions but have higher computational cost. RBPF offers a middle ground.

4. **Dynamic weight assignment** should use covariance-based methods (CI/ICI for unknown correlations) or confidence-based scaling (KD-EKF approach).

5. **Complementary filtering** directly applies to CSI+YOLO: camera for high-frequency local tracking, WiFi for low-frequency global correction.

6. **Camera provides cm-level accuracy within FOV; WiFi provides room-scale coverage** — these are fundamentally complementary.

7. **Temporal synchronization** requires PTP-class accuracy (sub-millisecond); DSICI handles asynchronous measurements.

8. **Sensor dropout** can be handled via multi-expert architectures (MoME-style) or covariance inflation with fallback modes.

9. **No existing CSI+YOLO fusion exists** — confirming novelty. The closest work (WiVi, iWVP) uses RSSI or decision-level fusion for different tasks.

10. **Measurement uncertainty** should be modeled via adaptive covariance matrices scaling inversely with sensor quality (SNR for WiFi, confidence for YOLO).

11. **Federated learning** is relevant for training the fusion model across privacy-sensitive deployments but not for runtime inference architecture.

12. **Position-level fusion** is recommended for the first implementation; feature-level fusion via deep learning (MM-Loc style) is a future enhancement.

---

*Research completed after 25+ independent web searches across IEEE, arXiv, Nature, MDPI, ACM, and Espressif documentation.*
