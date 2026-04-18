# CSI Tracking System — Cross-Verification Results

## High Confidence Findings (Confirmed by ≥2 dimensions + independent sources)

| Finding | Dimensions | Evidence |
|---------|-----------|----------|
| ESP32 CSI provides amplitude + phase per subcarrier, superior to RSSI | Dim01, Dim04, Dim05, Dim10 | Official esp-csi docs, multiple papers |
| ESP32 single antenna cannot directly run MUSIC (needs M>d elements) | Dim02, Dim10 | Confirmed by SpotFi paper, ESPARGOS project |
| No existing CSI+YOLO fusion for localization exists | Dim06, Dim07, Dim12 | WiVi fuses for activity recognition only; no position tracking fusion found |
| Phase coherence across unsynchronized ESP32s is unsolved | Dim03, Dim02, Dim05 | No pure-software solution achieves true cross-device phase sync |
| 100ms temporal smoothing reduces PDD variance but cannot compensate CFO | Dim03, Dim09 | PDD is zero-mean Gaussian; CFO is systematic offset |
| EKF fusion of WiFi + camera achieves 0.24-0.38m error | Dim07, Dim05 | Multiple independent papers confirm |
| ESP32-S3 achieves ~2m MAE for CSI ranging; ML fingerprinting gets 0.45m | Dim04, Dim05, Dim11 | Chaudhari et al. 2026, ESP32-S3 study |
| Hampel filter + PCA + phase sanitization is standard denoising pipeline | Dim09, Dim01 | Tsinghua tutorial, multiple CSI processing papers |
| YOLO11 achieves 20-50 FPS on CPU; ByteTrack best for tracking | Dim06, Dim08 | Ultralytics benchmarks, independent tests |
| Producer-consumer queue architecture is standard for real-time CSI | Dim08, Dim10 | ESPARGOS, pyespargos implementations |

## Medium Confidence Findings (1 authoritative source)

| Finding | Source | Evidence |
|---------|--------|----------|
| ESP32-C6 provides best CSI quality among ESP32 family | Espressif official ranking | C5 > C6 > C3 ≈ S3 > ESP32 |
| Unitary root-MUSIC reduces complexity 4× | Dim02 | Academic source, not widely implemented |
| Antenna switching can emulate arrays on ESP32 | Dim02, Dim10 | ESP-IDF supports up to 16 antennas via switch |
| Savitzky-Golay best preserves phase features during smoothing | Dim03, Dim09 | Filter theory well-established |
| Covariance Intersection (CI) handles unknown cross-correlations | Dim07 | CI method peer-reviewed |

## Conflict Zones

### Conflict 1: Fingerprinting vs Geometric Methods
- **Dim05, Dim11**: Fingerprinting achieves 0.45m median vs 1-3m for geometric methods on ESP32
- **Counter**: Geometric methods don't require calibration data collection
- **Resolution**: Both valid — fingerprinting for accuracy, geometric for flexibility

### Conflict 2: Can 100ms Smoothing Replace Hardware Sync?
- **Dim03**: Software cannot fully replace hardware sync; 100ms smoothing only handles PDD, not CFO
- **User's requirement**: Wants 100ms phase smoothing as primary sync mechanism
- **Resolution**: 100ms smoothing is viable for *differential* phase analysis and *amplitude-based* fusion, but absolute cross-device phase coherence requires CFO estimation per packet

### Conflict 3: MUSIC Feasibility on ESP32
- **Dim02**: Single antenna makes MUSIC impossible without external hardware
- **Counter**: Virtual arrays via subcarriers (SpotFi approach), multi-ESP arrays
- **Resolution**: MUSIC requires either (a) external antenna switch, (b) ESPARGOS-style phase-synced array, or (c) multi-ESP with time-division approach
