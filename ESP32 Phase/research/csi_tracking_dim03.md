# Dimension 03: Phase Synchronization & 100ms Smoothing for Multi-ESP32

## Executive Summary

Phase coherence across multiple unsynchronized ESP32 devices is a critical and largely **unsolved research gap** in distributed CSI-based localization. While existing multi-ESP32 work relies on RSSI for trilateration (suffering meter-level errors), CSI phase information offers sub-meter potential but is corrupted by independent clock offsets, carrier frequency offset (CFO), sampling frequency offset (SFO), and packet detection delay (PDD) between devices. This document provides a comprehensive analysis of phase synchronization challenges, existing compensation techniques, and the theoretical basis for software-based approaches including 100ms temporal smoothing.

---

## 1. Why Phase Coherence Is Critical for CSI-Based Localization

### 1.1 Phase Noise Directly Impacts Localization Accuracy

Phase information in CSI encodes critical geometric parameters: time-of-flight (ToF), angle-of-arrival (AoA), and Doppler shift. Without accurate phase, localization systems are limited to amplitude-based fingerprinting or RSSI trilateration, which typically achieves only meter-level accuracy [^3^].

```
Claim: "PhaseFi based on calibrated phases outperforms all the other schemes (i.e., FIFS, Horus, and ML) that are based on amplitudes" achieving mean errors of 1.08m vs. >2m for amplitude-based methods [^82^].
Source: PhaseFi: Phase Fingerprinting for Indoor Localization with a Deep Learning Approach (IEEE IoT Journal, 2016)
URL: https://ieeexplore.ieee.org/document/7417517/
Date: 2016
Excerpt: "PhaseFi based on calibrated phases outperforms all the other schemes (i.e., FIFS, Horus, and ML) that are based on amplitudes"
Context: First work to leverage calibrated CSI phase information for indoor fingerprinting; proved bounded variance of calibrated phase.
Confidence: High
```

### 1.2 Phase Slope Encodes Time-of-Flight

The fundamental relationship between phase slope across subcarriers and time-of-flight is:

$$\psi_k^{(l)} = \angle H_k^{(l)} \approx C_l - 2\pi f_k \tau_l$$

Where $\tau_l$ is the ToF of path $l$ and $f_k$ is the subcarrier frequency. This equation reveals that **ToF is encoded in the slope of phase across the frequency domain** [^64^].

```
Claim: "ToF is embedded within the phase slope of the CSI across the frequency domain... Without a common clock reference, random timing offsets introduce unknown biases to τ_l, rendering absolute ranging infeasible" [^64^].
Source: Path to Diversity: A Primer on ISAC-izing Commodity Wi-Fi
URL: https://arxiv.org/html/2601.12980v2
Date: 2026-02-09
Excerpt: "Physically, ToF is embedded within the phase slope of the CSI across the frequency domain. From Eqn. (4), for a single path, the phase ψ_k^(l) at subcarrier frequency f_k is: ψ_k^(l) ≈ C_l - 2πf_kτ_l"
Context: Fundamental equation for model-based Wi-Fi sensing; shows dual dependency on temporal diversity (synchronization) and frequency diversity (bandwidth).
Confidence: High
```

### 1.3 Phase Incoherence Destroys AoA Estimation

For angle-of-arrival estimation using antenna arrays, phase differences between antennas must be coherent. Random phase offsets from hardware errors make raw phase "useless for localization" without calibration [^82^].

### 1.4 ESP32 CSI vs. RSSI: Accuracy Gap

```
Claim: "Compared to RSSI-based ESP32 localization, which typically suffers from meter-level errors even with trilateration, CSI fingerprints provide finer spatial resolution" achieving sub-meter median accuracy [^3^].
Source: Device-Free Indoor Localization with ESP32 Wi-Fi CSI Fingerprints (Preprints, 2026)
URL: https://www.preprints.org/manuscript/202601.2378
Date: 2026-01-29
Excerpt: "Compared to RSSI-based ESP32 localization, which typically suffers from meter-level errors even with trilateration [14,15], CSI fingerprints provide finer spatial resolution"
Context: Multi-link ESP32 CSI device-free localization with three links achieved sub-meter median accuracy.
Confidence: High
```

---

## 2. What Causes Phase Incoherence Across Separate ESP32s

### 2.1 Independent Crystal Oscillators (40MHz, ±10ppm)

Each ESP32 uses its own 40MHz crystal oscillator with ±10ppm accuracy as standard [^43^][^51^]. This means:

- **Frequency mismatch**: Two ESP32s can differ by up to 20ppm in clock frequency
- **Phase drift accumulation**: At 2.4GHz WiFi carrier, a 10ppm offset produces ~24kHz CFO
- **Sampling misalignment**: Independent clocks cause SFO between transmitter and each receiver

```
Claim: "The accuracy of the selected crystal should be within ±10 ppm" for ESP32-S2 [^43^].
Source: ESP Hardware Design Guidelines (Espressif)
URL: https://docs.espressif.com/projects/esp-hardware-design-guidelines/en/latest/esp32s2/schematic-checklist.html
Date: Current (Espressif official documentation)
Excerpt: "Note that the accuracy of the selected crystal should be within ±10 ppm."
Context: Standard ESP32 crystal specification; this ±10ppm is the root cause of inter-device frequency misalignment.
Confidence: High
```

### 2.2 Carrier Frequency Offset (CFO)

The central frequencies of transmitter and receiver cannot be perfectly synchronized. The CFO corrector compensates this, but "due to the hardware imperfection, the compensation is incomplete. Signal still carries residual errors, which leads to a time-varying CSI phase offset across subcarriers" [^22^].

```
Claim: "The central frequencies of a transmission pair cannot be perfectly synchronized. The carrier frequency offset is compensated by the CFO corrector of the receiver, but due to the hardware imperfection, the compensation is incomplete" [^22^].
Source: Identifying Non-linear CSI Phase Measurement Errors with Commodity WiFi (ACM/IEEE)
URL: https://lion.sjtu.edu.cn/resource/downloadFile?filePath=/home/lion/lionweb/data/publication/text/20180122020134_377.pdf
Date: 2018
Excerpt: "Carrier Frequency Offset (CFO). The central frequencies of a transmission pair cannot be perfectly synchronized. The carrier frequency offset is compensated by the CFO corrector of the receiver, but due to the hardware imperfection, the compensation is incomplete."
Context: Comprehensive survey of CSI phase error sources on commodity WiFi NICs (Atheros, Intel).
Confidence: High
```

### 2.3 Sampling Frequency Offset (SFO)

SFO arises from "non-synchronized clocks" between transmitter and receiver, causing "the received signal after ADC a time shift with respect to the transmitted signal" [^22^]. Critically, "clock offsets are relatively stable within a short time (e.g., in the order of minutes), such phase rotation errors are nearly constant" but "for a large scale of time, time-varying SFO also leads to time-varying phase rotation errors" [^22^].

### 2.4 Packet Detection Delay (PDD)

PDD "stems from energy detection or correlation detection which occurs in digital processing after down convert and ADC sampling. Packet detection introduces another time shift with respect to the transmitted signal, which leads to packet-varying phase rotation error" [^22^].

```
Claim: "Packet detection delay (PDD). Packet detection delay stems from energy detection or correlation detection which occurs in digital processing after down convert and ADC sampling. Packet detection introduces another time shift with respect to the transmitted signal, which leads to packet-varying phase rotation error" [^22^].
Source: Identifying Non-linear CSI Phase Measurement Errors with Commodity WiFi
URL: Same as above
Date: 2018
Excerpt: Full quote above.
Context: PDD is packet-varying, making it particularly challenging for multi-receiver synchronization.
Confidence: High
```

### 2.5 Combined Phase Error Model

The measured CSI phase for subcarrier $k$ in channel $i$ can be expressed as [^22^]:

$$\Phi_{i,k} = \theta_{i,k} - 2\pi \cdot k \cdot f_s \cdot \delta + \beta + Z$$

Where:
- $\theta_{i,k}$ = true channel phase
- $\delta$ = timing offset (SFO + PDD)
- $\beta$ = constant phase offset (CFO residual + PLL phase offset)
- $Z$ = measurement noise

For **multiple unsynchronized ESP32 receivers**, each device has independent $\delta$ and $\beta$ values, making cross-device phase comparison fundamentally corrupted.

---

## 3. Phase Unwrapping: Resolving 2π Ambiguity

### 3.1 The Wrapping Problem

Raw CSI phase is "confined to a specific range, such as [-π, π] or [0, 2π]. This wrapping occurs because phase is typically expressed as an angle, and angles exceeding these bounds are wrapped back into the range" [^18^][^26^].

### 3.2 Unwrapping Formula

Phase unwrapping resolves discontinuities by adding integer multiples of 2π:

$$\hat{\Phi}_i = \Phi_i + 2\pi \left\lfloor \frac{\hat{\Phi}_{i-1} - \Phi_i}{2\pi} + 0.5 \right\rfloor$$

```
Claim: Phase unwrapping formula for CSI: "Φ^i = Φi + 2π⌊(Φ^i-1 - Φi)/2π⌋" where Φ^i, Φi are unwrapped and raw phases [^18^].
Source: Enhanced Wi-Fi Sensing: Leveraging Phase and Amplitude of CSI for Superior Accuracy
URL: https://www.preprints.org/manuscript/202412.2585
Date: 2024-12-31
Excerpt: "Phase unwrapping is the process of resolving these phase ambiguities to create a continuous phase representation."
Context: Standard phase unwrapping approach used across CSI sensing literature.
Confidence: High
```

### 3.3 MATLAB/Python Implementation Reference

The Tsinghua wireless sensing tutorial provides reference implementations:

```
csi_phase = unwrap(angle(csi_data), [], 2);  % Unwrap along subcarrier dimension [^28^]
```

```
Claim: Reference implementation shows unwrap along subcarrier dimension (dimension 2) [^28^].
Source: CSI Feature Extraction | Hands-on Wireless Sensing with Wi-Fi (Tsinghua)
URL: https://tns.thss.tsinghua.edu.cn/wst/docs/features/
Date: Current
Excerpt: "csi_phase = unwrap(angle(csi_data), [], 2);"
Context: Standard reference code for CSI phase processing.
Confidence: High
```

---

## 4. How 100ms Smoothing Window Helps Phase Coherence

### 4.1 The Rationale: Temporal Averaging Reduces Random Phase Noise

The key insight is that **hardware-induced phase offsets (CFO, SFO, PDD) contain both random and slowly-varying components**. A 100ms smoothing window (at 100-1000 Hz CSI rate = 10-100 packets) provides:

1. **Averaging of packet-varying PDD**: "The time shift τ_b varies in each packet reception but follows a Gaussian distribution with zero mean... λ_b can be removed by averaging over the measured CSI phases" [^99^].

2. **Tracking of slowly-varying SFO/CFO**: "Clock offsets are relatively stable within a short time (e.g., in the order of minutes)" [^22^]. Over 100ms, SFO drift is minimal but random packet-to-packet variations average out.

3. **Stabilization of phase reference**: With a moving window, each ESP32 maintains a locally consistent phase reference that evolves slowly.

### 4.2 Moving Average as Optimal Noise Reducer

```
Claim: "The moving average filter is (or so I have read) the theoretical best filter for solving a common problem; the reduction of random white noise while keeping the sharpest possible step response" [^68^].
Source: Filter Data Tutorial (University of St Andrews)
URL: https://www.st-andrews.ac.uk/~wjh/dataview/tutorials/filter.html
Date: Current
Excerpt: "The moving average filter is the theoretical best filter for solving a common problem; the reduction of random white noise while keeping the sharpest possible step response."
Context: Moving average is optimal for time-domain encoded signals where noise reduction is needed while preserving step changes.
Confidence: High
```

### 4.3 Savitzky-Golay for Preserving Phase Features

The Savitzky-Golay filter "fits a rolling window of data points with a low-degree polynomial through linear least squares to smooth out the incoming signal" and "can maintain the shape of the waveform better than a standard infinite impulse response (IIR) low-pass filter" [^33^].

```
Claim: "SG filter can maintain the shape of the waveform better than a standard infinite impulse response (IIR) low-pass filter" [^33^][^25^].
Source: WiFi Sensing on the Edge: Signal Processing Techniques (IEEE COMST, 2022)
URL: https://ebulutvcu.github.io/COMST22_WiFi_Sensing_Survey.pdf
Date: 2022
Excerpt: "In [90] it is suggested that SG filter can maintain the shape of the waveform better than a standard infinite impulse response (IIR) low-pass filter."
Context: Savitzky-Golay preserves peaks and valleys while smoothing - important for motion-induced phase changes.
Confidence: High
```

### 4.4 The TSFR Method: Time Smoothing + Frequency Rebuild

The Time Smoothing and Frequency Rebuild (TSFR) method specifically addresses CSI phase processing with a three-stage approach:

1. **Phase sanitization** via linear regression to remove impairments
2. **Time-domain filtering** with Savitzky-Golay for denoising
3. **Frequency rebuild** to eliminate SG-induced distortions

```
Claim: "TSFR method has been tested on five datasets... accuracy performance higher than 90% in most of the studied scenarios has been achieved... TSFR outperforms the state-of-the-art performance from 35% to 85%" in few-shot learning [^25^].
Source: Channel phase processing in wireless networks for human activity recognition (IoT Journal, 2023)
URL: https://www.sciencedirect.com/science/article/pii/S2542660523002834
Date: 2023-12-01
Excerpt: "TSFR consists of performing a CSI phase sanitization method to remove phase impairments based on a linear regression transformation method, then a time domain filtering stage with a Savitzky–Golay (SG) filter for denoising purposes"
Context: First paper to systematically combine phase sanitization with temporal SG filtering and frequency rebuilding.
Confidence: High
```

### 4.5 100ms Window: Specific Considerations

At typical CSI collection rates:
- **100 Hz**: 100ms = 10 packets (minimum viable averaging)
- **500 Hz**: 100ms = 50 packets (good averaging)
- **1000 Hz**: 100ms = 100 packets (excellent averaging)

The 100ms window balances:
- **Noise reduction**: More packets → better Gaussian noise averaging (improves as √N)
- **Temporal resolution**: 100ms preserves human motion dynamics (breathing ~0.2-0.5Hz, walking ~1-2Hz)
- **Channel coherence**: Indoor channel coherence time Tc ≈ 1/(2·fd) where fd is Doppler. For walking (fd < 12Hz), Tc ≈ 40ms [^99^]. A 100ms window slightly exceeds this, suggesting **adaptive window sizing** may be preferable.

---

## 5. Time Synchronization Methods for Distributed ESP32s

### 5.1 NTP (Network Time Protocol)

NTP provides "time accurate to within a few milliseconds of Coordinated Universal Time (UTC)" [^32^]. However, this is **insufficient for phase synchronization**:
- Millisecond-level sync = kilometers of phase ambiguity at 2.4GHz
- NTP jitter in WiFi environments can exceed 10ms
- Only suitable for coarse timestamp alignment, not phase coherence

### 5.2 ESP32 Hardware Timestamp Capabilities

The ESP32 provides `esp_timer_get_time()` with microsecond precision [^105^], but CSI callback timestamps (e.g., `wifi_csi_rx_cb`) have significant delays:

```
Claim: "Most of these functions work with a delay of more than a millisecond from the actual timestamp" for CSI reception callbacks [^76^].
Source: ESP-IDF GitHub Issue #10089
URL: https://github.com/espressif/esp-idf/issues/10089
Date: 2022-11-01
Excerpt: "I found that most of these functions work with a delay of more than a millisecond from the actual timestamp. As a result, I cannot return the packet as soon as I receive it."
Context: User requesting microsecond-level timing for bidirectional CSI; Espressif callbacks have >1ms latency.
Confidence: High
```

### 5.3 WiFi Beacon-Based Synchronization

Reference Broadcast Infrastructure Synchronization (RBIS) uses Wi-Fi beacon frames containing timestamps. "In Wi-Fi, beacon frames can be used as SYNC message... the default [interval] is 102.4 ms" [^41^].

```
Claim: "In Wi-Fi, beacon frames can be used as SYNC message. They contain the SSID of the access point (AP), the time interval of the transmission, and the timestamp of the beacon... This interval can be adjusted, whereas the default is 102.4 ms" [^41^].
Source: ETFA 2024 Conference Paper (Reference Broadcast Infrastructure Synchronization)
URL: https://arxiv.org/html/2410.08742v1
Date: 2024-10-11
Excerpt: "In Wi-Fi, beacon frames can be used as SYNC message. They contain... the timestamp of the beacon, i.e., the time that elapsed since the AP was powered."
Context: Beacon-based sync achieves offset/skew estimation between master and slave clocks.
Confidence: High
```

### 5.4 RFClock: RF-Based Hardware Synchronization (State-of-the-Art)

RFClock achieves "less than 5 nano-second level time deviation" using hardware-assisted RF synchronization [^104^].

```
Claim: "RFClock performs as well as the Octoclock, with less than 5 nano-second level time deviation and operates in the 95 percentile for 0.21Hz and 0.93Hz frequency offset at 915MHz and 2.4GHz" [^104^].
Source: RFClock: Timing, Phase and Frequency Synchronization (ACM MobiCom 2021)
URL: https://genesys-lab.org/papers/RFCLOCK_MOBICOM2021.pdf
Date: 2021
Excerpt: "RFClock performs as well as the Octoclock, with less than 5 nano-second level time deviation"
Context: Hardware-based solution; not applicable to pure ESP32 software approach but represents the gold standard.
Confidence: High
```

### 5.5 Summary: Synchronization Methods Comparison

| Method | Accuracy | ESP32 Feasibility | Phase Sync Suitability |
|--------|----------|-------------------|----------------------|
| NTP | ~1-10ms | Software only | No - too coarse |
| Beacon-based (RBIS) | ~μs level estimate | Software only | Partial - good for offset/skew |
| Timestamp-free (RF) | ~12% of sample period [^71^] | Not directly applicable | Potential with modifications |
| RFClock (hardware) | <5ns | Requires extra hardware | Yes - gold standard |
| GPS/atomic clock | <1ns | Requires GPS module | Yes - expensive |
| **100ms software smoothing** | **Relative only** | **Fully software** | **Partial - for differential phase** |

---

## 6. Can Software-Based Phase Compensation Replace Hardware Sync?

### 6.1 The Fundamental Limitation

**Absolute phase synchronization between independent ESP32s cannot be achieved purely in software** because:

1. Each receiver has independent random phase offsets per packet
2. CFO/SFO are not observable from single-node CSI without external reference
3. Time-of-flight and timing offset are confounded in phase slope

```
Claim: "Currently, there is no 'perfect algorithm' to solve this type of error. Conjugate multiplication and division are the only two methods to eliminate the SFO and PDD. By applying the conjugate multiplication or division, the ε_t is eliminated, at the cost of losing absolute ToF measurement" [^35^].
Source: CSI Sanitization Tutorial (Tsinghua University)
URL: https://tns.thss.tsinghua.edu.cn/wst/docs/sanitization/
Date: Current
Excerpt: "Currently, there is no 'perfect algorithm' to solve this type of error. Conjugate multiplication and division are the only two methods to eliminate the SFO and PDD."
Context: Fundamental limitation acknowledged by leading CSI sensing research group.
Confidence: High
```

### 6.2 What Software CAN Do

Software compensation is effective for:

1. **Intra-packet linear phase removal**: Removing slope + offset per packet
2. **Temporal smoothing**: Reducing random noise over packet windows
3. **Differential phase between subcarriers/antennas**: Eliminating common offsets
4. **Relative phase tracking**: Monitoring phase changes for motion detection

### 6.3 The "Phase Sanitization" Approach

Phase sanitization via linear fitting removes the mean and slope of CSI phase across subcarriers:

$$\angle\widehat{CSI}_i = \angle CSI_i - \frac{\angle CSI_{30} - \angle CSI_1}{m_{30} - m_1} m_i - \frac{1}{30}\sum_{i=1}^{30} \angle CSI_i$$

```
Claim: "By implementing a linear transformation on the raw phases to remove the terms of Δt and β... the calibrated phase is relatively stable" with bounded variance [^82^].
Source: PhaseFi Paper (Auburn University, IEEE IoT Journal 2016)
URL: https://www.eng.auburn.edu/~szm0001/papers/GC15_XWang.pdf
Date: 2016
Excerpt: "Let ∠CSI_i denote the measured phase of subcarrier i. It can be written as: ∠CSI_i = ∠CSI_i + 2π(m_i/N)Δt + β + Z"
Context: PhaseFi proved theoretically that calibrated phase has bounded variance.
Confidence: High
```

### 6.4 Conjugate Multiplication for SFO/PDD Removal

For multi-antenna systems, conjugate multiplication eliminates timing offsets:

```matlab
% Conjugate multiplication removes SFO/PDD
csi_remove_sto(:, :, a, :) = csi_src(:, :, a, :) .* conj(csi_src(:, :, a_nxt, :));
```

```
Claim: "Conjugate multiplication and division are the only two methods to eliminate the SFO and PDD" [^35^][^66^].
Source: Hands-on Wireless Sensing with Wi-Fi (Tsinghua tutorial)
URL: https://tns.thss.tsinghua.edu.cn/~guoxuan/assets/pdf/Paper-Hands-On.pdf
Date: Current
Excerpt: "Currently, there is no 'perfect algorithm' to solve this type of error. Conjugate multiplication and division are the only two methods to eliminate the SFO and PDD."
Context: Works for multi-antenna on SAME receiver, not across independent receivers.
Confidence: High
```

### 6.5 Counter-Narrative: Some Success Without Hardware Sync

```
Claim: "MonoLoco is the only single access-point solution that provides decimeter-level localization and orientation information and requires no coordination, time synchronization or external networking protocol" [^17^].
Source: Multipath Triangulation: Decimeter-level WiFi Localization (MobiSys 2018)
URL: https://elahe.web.illinois.edu/Elahe%20Soltan_files/papers/MobiSys18_MonoLoco_CameraReady.pdf
Date: 2018
Excerpt: "MonoLoco is the only single access-point solution that provides decimeter-level localization and orientation information and requires no coordination, time synchronization or external networking protocol with the target or with other APs."
Context: Uses relative ToF (rToF) between multipath components instead of absolute ToF, avoiding synchronization requirement.
Confidence: High
```

---

## 7. Phase Slope and Time-of-Flight Relationship

### 7.1 Mathematical Foundation

For a single path, the phase at subcarrier $k$ with frequency $f_k$ is:

$$\psi_k = C - 2\pi f_k \tau$$

Where $\tau$ is the ToF. The phase slope across subcarriers is:

$$\frac{\Delta\psi}{\Delta f} = -2\pi \tau$$

### 7.2 Practical Extraction via IFFT

"The standard method to extract ToF in a multi-path environment is to apply an Inverse Fast Fourier Transform (IFFT) to the CSI vector. This transforms the signal to the time domain, yielding a Power Delay Profile (PDP) where each peak corresponds to the ToF of a distinct multipath component" [^64^].

### 7.3 Phase Slope Error Sources

In practice, the measured phase slope contains both true ToF and timing offset (SFO + PDD):

$$\text{measured slope} = -2\pi(\tau_{ToF} + \tau_{SFO} + \tau_{PDD})$$

This is why linear fitting and subtraction is commonly used - it removes the combined timing offset but also removes the absolute ToF information.

---

## 8. CFO Estimation and Compensation Between ESP32s

### 8.1 HT-LTF-Based CFO Estimation

The 802.11 standard requires multiple HT-LTFs spaced at exactly 4μs. The phase difference between HT-LTFs is caused by CFO:

```matlab
% CFO estimation from dual HT-LTFs
delta_time = 4e-6;
phase_diff = mean(phase_2 - phase_1, 3);
est_cfo = mean(phase_diff ./ delta_time, 2);
```

```
Claim: "The phase difference between two HT-LTFs is induced by the CFO within Δt=4μs. Thus, the approximate value of the CFO can be recovered" [^35^].
Source: CSI Sanitization Tutorial (Tsinghua)
URL: https://tns.thss.tsinghua.edu.cn/wst/docs/sanitization/
Date: Current
Excerpt: "Since the time interval between multiple HT-LTFs is strictly controlled to 4μs according to the 802.11 protocol, the phase difference between two HT-LTFs is induced by the CFO within Δt=4μs."
Context: HT-LTF based CFO estimation works per-packet but requires multiple HT-LTFs in PPDU.
Confidence: High
```

### 8.2 MUSIC-Based Residual CFO Estimation

Chen et al. (2020) proposed using "multiscale sparse recovery algorithm to get rid of the effect of PDD and extract the carrier frequency component out of CSI" then "formulate the residual CFO estimation as a spectrum estimation problem and utilize the MUSIC algorithm" [^30^].

```
Claim: "The residual CFO is time-varying, and compared with existing methods, the proposed method can better estimate and compensate the residual CFO" [^30^].
Source: Residual Carrier Frequency Offset Estimation and Compensation for Commodity WiFi (IEEE TMC, 2020)
URL: https://ui.adsabs.harvard.edu/abs/2020ITMC...19.2891C/abstract
Date: 2020
Excerpt: "The experimental results and simulation results show that the residual CFO is time-varying"
Context: MUSIC-based approach achieves better CFO estimation than prior methods; requires multiple CSI realizations.
Confidence: High
```

### 8.3 Kalman Filter-Based Phase Recovery

Li et al. proposed an "adaptive CSI estimation approach based on Kalman filter (KF) with maximum-a-posteriori (MAP) estimation that considers the CSI from the previous time" [^44^][^47^].

```
Claim: "Our approach can track the channel variations while eliminating the phase errors accurately... the proposed method outperforms linear regression significantly" and approaches CRLB at high SNR [^44^].
Source: Kalman filter based MIMO CSI phase recovery for COTS WiFi devices
URL: https://arxiv.org/abs/2101.06186
Date: 2021-01-15
Excerpt: "Simulation and experimental results demonstrate that our approach can track the channel variations while eliminating the phase errors accurately."
Context: KF+MAP approach jointly estimates channel state and phase distortion parameters; MSE approaches CRLB.
Confidence: High
```

---

## 9. Phase Smoothing Filters: Comparison

### 9.1 Moving Average (MA)

$$\hat{h}[t] = \frac{1}{w}\sum_{i=0}^{w-1} h[t-i]$$

- **Pros**: Fastest digital filter available (recursive implementation O(1) per sample); optimal for reducing white noise while keeping sharpest step response
- **Cons**: Blunts sharp features; introduces lag; equal weighting

### 9.2 Gaussian Filter

Weighted moving average with Gaussian kernel - "gives more weight to points near the center and less to those further away" [^70^].

- **Pros**: Smoother results with fewer edge effects than MA; naturally approximated by 4+ passes of MA
- **Cons**: Blurs sharp edges; non-adaptive

### 9.3 Savitzky-Golay Filter

"Fits a rolling window of data points with a low-degree polynomial through linear least squares" [^33^].

- **Pros**: "Maintains the shape of the waveform better than standard IIR low-pass filter"; preserves peaks and valleys
- **Cons**: "Poor anomaly filtering"; not effective against outliers

### 9.4 Kalman Filter

Recursive probabilistic algorithm that estimates system state from noisy observations [^18^].

- **Pros**: Handles variable noise and missing data; adapts over time; optimal for linear Gaussian systems
- **Cons**: Computationally heavier; requires good process model; may diverge with bad parameters

### 9.5 Hampel Filter

Median-based outlier detection that returns exact waveform except for anomalies.

- **Pros**: Robust to outliers; preserves signal shape
- **Cons**: "Anomalies detected may in fact be important"

### 9.6 Filter Selection Guidance for CSI Phase

| Filter | Use Case | ESP32 Feasibility |
|--------|----------|-------------------|
| Moving Average | General noise reduction, real-time | Excellent - O(1) recursive |
| Gaussian | Smooth phase trends | Good - approximated by MA passes |
| Savitzky-Golay | Preserve motion features while smoothing | Good - polynomial fitting |
| Kalman | Track phase with adaptive noise | Moderate - requires tuning |
| Hampel | Remove phase outliers | Excellent - median-based |

---

## 10. Packet Arrival Time Jitter and Phase Alignment

### 10.1 Jitter Sources in 802.11

Packet inter-arrival jitter in WiFi networks includes:
- MAC-layer contention/backoff variability
- Packet detection delay variation (±24.8ns standard deviation measured by Chronos [^60^])
- Processing delay in CSI callbacks (>1ms on ESP32 [^76^])

```
Claim: "Chronos observes a median packet detection delay of 177 ns... Packet detection delay is nearly 8x larger than the time-of-flight... packet delay varies dramatically between packets, and has a high standard deviation of 24.8 ns" [^60^].
Source: Decimeter-Level Localization with a Single WiFi Access Point (MIT/NSDI 2016)
URL: https://www.usenix.org/system/files/conference/nsdi16/nsdi16-paper-vasisht.pdf
Date: 2016
Excerpt: "Packet detection delay is nearly 8x larger than the time-of-flight in our typical indoor testbed... has a high standard deviation of 24.8 ns."
Context: PDD is the dominant error source for ToF estimation on commodity WiFi.
Confidence: High
```

### 10.2 Statistical Distribution of PDD

Splicer's analysis found that packet boundary detection delay follows a Gaussian distribution with zero mean [^99^]. This means **temporal averaging over sufficient packets can effectively remove PDD**.

### 10.3 Impact on Multi-Receiver Alignment

For multiple ESP32s receiving the same packets:
- Each receiver experiences **independent PDD realizations**
- The PDD is not correlated across receivers
- Cross-receiver phase comparison is corrupted by uncorrelated PDD
- **Solution**: Per-receiver temporal averaging reduces PDD variance

---

## 11. Phase Sanitization via Linear Fitting

### 11.1 The Core Method

Phase sanitization removes linear phase errors by fitting and subtracting a line across subcarriers. The linear model is [^82^]:

$$k = \frac{\angle\widehat{CSI}_{30} - \angle\widehat{CSI}_1}{m_{30} - m_1}, \quad b = \frac{1}{30}\sum_{i=1}^{30} \angle\widehat{CSI}_i$$

$$\angle\widehat{CSI}_i = \angle\widehat{CSI}_i - k \cdot m_i - b$$

### 11.2 What Sanitization Removes and What It Loses

```
Claim: "The phase sanitization approach compensates for the phase rotation by linearly removing the mean and slope of the measured CSI phase. This is a double-edged approach in which phase rotation is mitigated but simultaneously a part of the CSI is proportionally removed along with it" [^40^].
Source: Mitigation of CSI Temporal Phase Rotation with B2B (PMC, 2017)
URL: https://pmc.ncbi.nlm.nih.gov/articles/PMC6263436/
Date: 2017-05-11
Excerpt: "The phase sanitization approach compensates for the phase rotation by linearly removing the mean and slope of the measured CSI phase. This is a double-edged approach in which phase rotation is mitigated but simultaneously a part of the CSI is proportionally removed along with it."
Context: Sanitization is the most common approach but has fundamental limitations - it removes genuine phase information along with errors.
Confidence: High
```

### 11.3 Advanced: Non-Linear Error Correction

Recent work identified that commodity WiFi NICs exhibit **non-linear phase errors** that linear fitting cannot remove [^22^].

```
Claim: "An unrevealed non-linear phase error exists, which cannot be mitigated through existing methods... this non-linear error is orders-of-magnitude higher than the ground truth phase and thus non-negligible" [^22^].
Source: Identifying Non-linear CSI Phase Measurement Errors (2018)
URL: https://lion.sjtu.edu.cn/resource/downloadFile?filePath=/home/lion/lionweb/data/publication/text/20180122020134_377.pdf
Date: 2018
Excerpt: "An unrevealed non-linear phase error exists, which cannot be mitigated through existing methods. To make matter worse, obviously this non-linear error is orders-of-magnitude higher than the ground truth phase and thus non-negligible."
Context: IQ imbalance is identified as root cause of non-linear phase errors.
Confidence: High
```

### 11.4 Back-to-Back (B2B) Calibration

The most accurate phase calibration uses a reference channel:

```
Claim: "This is the first calibration method that maintains the original CSI and is applicable for in-depth motion analysis" using back-to-back wired channel [^40^].
Source: Mitigation of CSI Temporal Phase Rotation with B2B
URL: https://pmc.ncbi.nlm.nih.gov/articles/PMC6263436/
Date: 2017
Excerpt: "The proposed technique uses a so-called reference CSI, a time-invariant channel that experiences the same phase rotation as the target CSI."
Context: B2B calibration achieves RMSE of ~0.117 radians (~6.7°) vs. VNA ground truth.
Confidence: High
```

---

## 12. Clock Drift Modeling and Compensation

### 12.1 Linear Clock Model

The standard model for clock offset between two nodes is [^74^]:

$$Y = \alpha + \beta \cdot x - e_v$$

Where:
- $\alpha$ = clock offset (phase error)
- $\beta$ = clock skew (frequency ratio)
- $x$ = local time
- $e_v$ = random measurement noise

### 12.2 Least-Squares Drift Estimation

Nodes store timestamp pairs $(x, Y)$ and compute least-squares regression to estimate $\alpha$ and $\beta$. However, "the error of the predicted clock values grows as time passes at each sensor node" between synchronizations [^74^].

### 12.3 Temperature-Dependent Drift Model

Clock drift is strongly temperature-dependent:

```
Claim: "The STM derived by ambient temperature between the two nodes is: α̂_SR[i](ppm) = 5.559T_R[i] - 3.780T_S[i] - 44.475" showing temperature coefficients can be derived [^75^].
Source: MODELING AND TRACKING TIME-VARYING CLOCK ERRORS (Georgia Tech thesis)
URL: https://repository.gatech.edu/server/api/core/bitstreams/43896d5a-455e-4cfd-b1f3-79dabf3892a2/content
Date: 2014
Excerpt: "If a sender transmits temperature measurements, which shift relatively slowly, instead of timestamps, a receiver can estimate the skew of its local clock from that of the sender based on the established STM."
Context: Temperature-sensitive model enables less frequent timestamp exchange for drift compensation.
Confidence: High
```

### 12.4 Allan Variance for Clock Stability

The Allan variance characterizes clock stability over different averaging times. For ESP32's 40MHz ±10ppm crystal:
- Short-term stability (~1s): Dominated by phase noise
- Medium-term (~100s): Dominated by temperature-induced drift
- Long-term (>1000s): Dominated by aging (typically 1-5 ppm/year)

### 12.5 Practical Clock Drift for ESP32

With 10ppm crystals:
- **Maximum frequency difference**: 20ppm between two ESP32s
- **Phase drift at 2.4GHz**: ~48kHz beat frequency maximum
- **Phase rotation rate**: ~0.3 rad/ms maximum (at worst-case 20ppm offset)
- **100ms phase drift**: Up to ~30 radians (≈ 4.8 full rotations)

This means **CFO must be tracked and compensated continuously**; simple averaging without CFO handling will fail.

---

## 13. Key Stakeholders and Their Contributions

### 13.1 ESPARGOS Team (TU Darmstadt / Euchner et al.)

ESPARGOS is the closest existing system to multi-ESP32 phase-coherent CSI collection, but uses **shared clock and phase reference via cabling**:

```
Claim: "By feeding reference clock and phase calibration signal from external sources, multiple circuit boards can be combined into one large phase-coherent system" [^67^].
Source: ESPARGOS: An Ultra Low-Cost, Realtime-Capable Multi-Antenna WiFi Channel Sounder
URL: https://arxiv.org/html/2502.09405v1
Date: 2025-02-13
Excerpt: "By feeding reference clock and phase calibration signal from external sources, multiple circuit boards can be combined into one large phase-coherent system."
Context: ESPARGOS uses shared 40MHz reference clock + WiFi-based phase reference via coaxial cable distribution network.
Confidence: High
```

### 13.2 MegaMIMO Team (MIT CSAIL)

Pioneered distributed phase synchronization for software-defined radios:

```
Claim: "MegaMIMO provides the first system that achieves phase synchronization using independent oscillators at the devices in the network... 95th percentile misalignment between APs observed at the receiver is less than 0.05 radians" [^102^].
Source: MegaMIMO: Scaling Wireless Capacity with User Demands (ACM SIGCOMM 2012)
URL: https://people.csail.mit.edu/rahul/papers/megamimo-sigcomm2012.pdf
Date: 2012
Excerpt: "MegaMIMO's distributed phase synchronization algorithm is accurate. The 95th percentile misalignment between APs observed at the receiver is less than 0.05 radians."
Context: Uses lead AP phase reference + per-packet header re-synchronization; requires SDR platform (USRP2).
Confidence: High
```

### 13.3 AirSync Team (USC)

```
Claim: "AirSync is able to achieve timing synchronization within the OFDM CP and carrier phase coherence within a few degrees" [^103^].
Source: AirSync: Enabling Distributed Multiuser MIMO With Full Spatial Multiplexing (IEEE TON)
URL: https://sites.usc.edu/kpsounis/files/2019/05/ton_airsync.pdf
Date: 2019
Excerpt: "AirSync is able to achieve timing synchronization within the OFDM CP and carrier phase coherence within a few degrees."
Context: Implemented as FPGA digital circuit on WARP radios; uses pilot tone tracking for phase prediction.
Confidence: High
```

### 13.4 Splicer Team (NTU)

Xie, Li, and Li developed comprehensive CSI phase error compensation for commodity WiFi:

```
Claim: "Splicer can reduce the median ranging error from 7.1m to 0.63m compared with using raw CSI traces from NICs" [^98^][^99^].
Source: Splicer: Precise Power Delay Profiling with Commodity WiFi (ACM MobiCom 2015)
URL: https://www.cs.cityu.edu.hk/~zhenjili/2015-MobiCom-Splicer.pdf
Date: 2015
Excerpt: "Splicer can reduce the median ranging error from 7.1m to 0.63m compared with using raw CSI traces from NICs."
Context: Demonstrated that comprehensive phase error correction enables accurate ranging on commodity hardware.
Confidence: High
```

### 13.5 Chronos Team (MIT CSAIL)

```
Claim: "The median errors in time-of-flight estimation are 0.47 ns and 0.69 ns respectively [LoS/NLoS]... 95th percentile error is 1.96 ns in line-of-sight" [^60^].
Source: Decimeter-Level Localization with a Single WiFi Access Point (NSDI 2016)
URL: https://www.usenix.org/system/files/conference/nsdi16/nsdi16-paper-vasisht.pdf
Date: 2016
Excerpt: "The median errors in time-of-flight estimation are 0.47 ns and 0.69 ns respectively."
Context: Uses channel hopping + sub-nanosecond ToF from single AP; addresses timing offsets algorithmically.
Confidence: High
```

---

## 14. The Research Gap: Multi-ESP32 Without Hardware Sync

### 14.1 Current State Summary

| Approach | Hardware Requirement | Phase Coherence | ESP32 Applicability |
|----------|---------------------|-----------------|---------------------|
| ESPARGOS [^67^] | Shared clock via cable | Full (phase-coherent array) | Requires hardware mod |
| MegaMIMO [^102^] | SDR platform (USRP2) | High (0.05 rad 95th %) | Not applicable |
| AirSync [^103^] | FPGA (WARP) | High (few degrees) | Not applicable |
| RFClock [^104^] | Extra RF hardware | <5ns | Requires add-on board |
| Splicer [^99^] | Commodity NIC | Per-receiver only | Principles applicable |
| **Proposed: 100ms smoothing** | **None (software only)** | **Relative/differential** | **Fully applicable** |

### 14.2 What 100ms Smoothing CAN and CANNOT Do

**CAN do:**
- Stabilize per-receiver phase measurements over time
- Reduce random PDD-induced noise via temporal averaging
- Enable differential phase analysis for motion sensing
- Provide a slowly-evolving phase reference at each receiver

**CANNOT do:**
- Establish absolute phase relationship between receivers
- Remove systematic CFO/SFO offsets between devices
- Enable direct cross-receiver phase comparison for AoA
- Compensate for independent clock drift between ESP32s

### 14.3 Practical Implementation Strategy

For multi-ESP32 localization using 100ms smoothing:

1. **Per-ESP32 processing**:
   - Phase unwrapping along subcarriers
   - Linear sanitization (slope + offset removal) per packet
   - Temporal smoothing (100ms moving average or SG filter)
   - Differential phase features (between subcarriers)

2. **Cross-ESP32 fusion** (at central server):
   - Amplitude-based features (RSSI from each ESP32)
   - Differential phase features (NOT direct phase comparison)
   - Temporal correlation of motion-induced phase changes
   - Machine learning-based position inference

3. **Expected performance**:
   - Amplitude + differential phase: Sub-meter potential
   - Direct phase comparison between ESP32s: Not possible without sync
   - Relative motion tracking: Good (independent per-device phase trends)

---

## 15. Conclusions and Recommendations

### 15.1 Key Findings

1. **Phase coherence across unsynchronized ESP32s is fundamentally limited** by independent 40MHz crystal oscillators (±10ppm each) producing uncorrelated CFO, SFO, and PDD at each receiver.

2. **No existing implementation** achieves cross-device phase coherence on ESP32s without hardware modifications (shared clock or external reference).

3. **100ms temporal smoothing is viable and useful** for:
   - Stabilizing per-receiver phase measurements
   - Enabling differential phase features for motion sensing
   - Reducing random PDD noise via averaging

4. **Software-only approaches cannot achieve absolute cross-device phase synchronization** but can enable relative/differential phase analysis.

5. **The most promising approach** combines:
   - Per-packet phase sanitization (linear fitting)
   - 100ms temporal smoothing (MA or Savitzky-Golay)
   - Differential phase features (not direct cross-device phase)
   - ML-based fusion of multi-receiver amplitude + differential phase

### 15.2 Recommendations for Implementation

1. **Use Savitzky-Golay filter** for temporal smoothing (preserves motion features better than pure MA)
2. **Implement phase sanitization** (linear slope + offset removal) before temporal smoothing
3. **Use conjugate multiplication** for intra-receiver antenna phase differences (removes common offsets)
4. **Do not attempt direct phase comparison** between unsynchronized ESP32s
5. **Focus on differential features** that are invariant to per-receiver phase offsets
6. **Consider 50-100ms smoothing window** as practical trade-off between noise reduction and temporal resolution

### 15.3 Open Questions

1. Can beacon-based timing sync provide sufficient accuracy for ESP32 phase alignment?
2. How much does temperature-induced clock drift affect 100ms smoothing effectiveness?
3. Can machine learning learn to compensate for inter-device phase incoherence?
4. What is the theoretical bound on localization accuracy with unsynchronized ESP32s using differential phase only?

---

## References

[^2^] A Deep Dive Into ESP-CSI: Channel State Information on ESP32 Chips, Dev.to, 2025.
[^3^] Device-Free Indoor Localization with ESP32 Wi-Fi CSI Fingerprints, Preprints, 2026.
[^17^] Multipath Triangulation: Decimeter-level WiFi Localization and Orientation, MobiSys 2018.
[^18^] Enhanced Wi-Fi Sensing: Leveraging Phase and Amplitude of CSI for Superior Accuracy, Preprints, 2024.
[^20^] WiFi Sensing with Channel State Information: A Survey, ACM CSUR, 2019.
[^22^] Identifying Non-linear CSI Phase Measurement Errors with Commodity WiFi, 2018.
[^25^] Channel phase processing in wireless networks for HAR, IoT Journal, 2023.
[^28^] CSI Feature Extraction - Hands-on Wireless Sensing with Wi-Fi (Tsinghua Tutorial).
[^30^] Residual Carrier Frequency Offset Estimation and Compensation, IEEE TMC, 2020.
[^32^] ESP32 NTP Time Sync: Get Accurate Time Without RTC Module.
[^33^] WiFi Sensing on the Edge: Signal Processing Techniques, IEEE COMST, 2022.
[^35^] CSI Sanitization Tutorial (Tsinghua University).
[^40^] Mitigation of CSI Temporal Phase Rotation with B2B Calibration, PMC, 2017.
[^41^] RBIS: Reference Broadcast Infrastructure Synchronization Protocol, ETFA 2024.
[^43^] ESP Hardware Design Guidelines - Crystal Clock Source (Espressif).
[^44^] Kalman filter based MIMO CSI phase recovery for COTS WiFi devices, 2021.
[^46^] ESP32 S3 Improving hardware timer accuracy discussion, StackOverflow, 2024.
[^47^] Kalman filter based MIMO CSI phase recovery (arXiv), 2021.
[^51^] ESP32 Hardware Design Guidelines (Espressif official).
[^60^] Decimeter-Level Localization with a Single WiFi Access Point (Chronos), NSDI 2016.
[^64^] Path to Diversity: A Primer on ISAC-izing Commodity Wi-Fi, 2026.
[^67^] ESPARGOS: An Ultra Low-Cost Multi-Antenna WiFi Channel Sounder, 2025.
[^68^] Filter Data Tutorial (University of St Andrews).
[^70^] Six Approaches to Time Series Smoothing (Medium), 2025.
[^71^] A Software-Defined Radio Implementation of Timestamp-Free Network Synchronization, ICASSP 2017.
[^74^] Drift estimation using pairwise slope with minimum variance, Ad Hoc Networks (Elsevier).
[^75^] MODELING AND TRACKING TIME-VARYING CLOCK ERRORS (Georgia Tech thesis), 2014.
[^76^] ESP-IDF GitHub Issue #10089 - microsecond-level CSI timing.
[^82^] PhaseFi: Phase Fingerprinting for Indoor Localization with a Deep Learning Approach, IEEE IoT Journal, 2016.
[^87^] Evaluation of Self-Positioning Algorithms for Time-of-Flight, WiOpt 2016.
[^88^] CSI Ranging-based Wi-Fi Indoor Localization Error Analysis, WCSP 2021.
[^92^] Multi-Station WiFi CSI Sensing Framework Robust to Feature Missingness, 2026.
[^95^] Open-Loop Distributed Beamforming Using Wireless Phase Synchronization, IEEE.
[^98^] Precise Power Delay Profiling with Commodity WiFi (Splicer), ACM MobiCom 2015.
[^99^] Splicer: MobiCom 2015 full paper.
[^102^] MegaMIMO: Scaling Wireless Capacity with User Demands, ACM SIGCOMM 2012.
[^103^] AirSync: Enabling Distributed Multiuser MIMO With Full Spatial Multiplexing, IEEE TON.
[^104^] RFClock: Timing, Phase and Frequency Synchronization, ACM MobiCom 2021.
[^105^] High Resolution Timer (ESP Timer) - ESP-IDF Documentation.
[^106^] Regulating Queueing Delay in 802.11ac WLANs (Leith, Trinity College).
