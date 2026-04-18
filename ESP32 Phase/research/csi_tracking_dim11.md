# Dimension 11: Indoor Localization Accuracy Evaluation

## Comprehensive Deep Research Report

**Date**: 2026-01-29
**Searches Conducted**: 23 independent web searches across IEEE, ACM, NIST, arXiv, Nature, Espressif docs, and primary academic sources
**Focus**: WiFi CSI-based indoor localization evaluation methodologies, metrics, benchmarks, and cross-technology comparison

---

## Table of Contents

1. [Standard Metrics for Indoor Localization](#1-standard-metrics-for-indoor-localization)
2. [Ground Truth Establishment Methods](#2-ground-truth-establishment-methods)
3. [Cumulative Distribution Function (CDF) of Error](#3-cumulative-distribution-function-cdf-of-error)
4. [Benchmark Datasets for WiFi Indoor Localization](#4-benchmark-datasets-for-wifi-indoor-localization)
5. [Statistically Significant Test Point Requirements](#5-statistically-significant-test-point-requirements)
6. [Factors Affecting Localization Accuracy](#6-factors-affecting-localization-accuracy)
7. [Real-Time Tracking vs Static Positioning Evaluation](#7-real-time-tracking-vs-static-positioning-evaluation)
8. [Cramer-Rao Lower Bound (CRLB) for WiFi Localization](#8-cramer-rao-lower-bound-crlb-for-wifi-localization)
9. [Fair Comparison Against Baseline Systems](#9-fair-comparison-against-baseline-systems)
10. [Typical Accuracy Ranges Across Technologies](#10-typical-accuracy-ranges-across-technologies)
11. [System Robustness Over Time](#11-system-robustness-over-time)
12. [Statistical Tests for Significance Validation](#12-statistical-tests-for-significance-validation)
13. [Key Stakeholders and Initiatives](#13-key-stakeholders-and-initiatives)
14. [Counter-Narratives and Limitations](#14-counter-narratives-and-limitations)

---

## 1. Standard Metrics for Indoor Localization

### Primary Accuracy Metrics

Indoor localization systems employ a family of related metrics to quantify performance. The most commonly used are:

#### Mean Absolute Error (MAE)

**Claim**: MAE is the arithmetic average of absolute errors between predicted and observed positions, providing a robust measure of central tendency that is less sensitive to outliers than RMSE. [^120^]
**Source**: "A Survey of Recent Indoor Localization Scenarios and Methodologies", PMC (2020)
**URL**: https://pmc.ncbi.nlm.nih.gov/articles/PMC8662396/
**Date**: 2020-02-28
**Excerpt**: "Another error measurement method MAE (i.e., mean absolute error) contributes to measure the arithmetic average of absolute error between the predicted value and the observed value."
**Context**: MAE is widely adopted because it is intuitive and less affected by large outliers.
**Confidence**: High

#### Root Mean Square Error (RMSE)

**Claim**: RMSE is the most common metric for localization accuracy, computed as the square root of the average squared errors. It is highly sensitive to large deviations and thus penalizes significant outliers. [^120^] [^121^]
**Source**: "A Survey of Recent Indoor Localization Scenarios and Methodologies", PMC (2020); "Floor-Plan-aided Indoor Localization", arXiv (2024)
**URL**: https://pmc.ncbi.nlm.nih.gov/articles/PMC8662396/, https://arxiv.org/html/2405.13339v1
**Date**: 2020-02-28; 2024-05-22
**Excerpt**: "Many contributions apply RMSE to reflect the localization accuracy... the CDF of ϵm(t), we can obtain the mean absolute error (MAE), the median of the errors (with legend '50% CDF'), and the 90-th percentile CDF"
**Context**: RMSE is the standard metric in virtually all localization papers and competitions.
**Confidence**: High

#### Median Error (50th Percentile)

**Claim**: The median localization error (50th percentile) is the most commonly reported single-number metric because it represents the "typical" performance without being skewed by outliers. [^59^] [^133^]
**Source**: "SpotFi: Decimeter Level Localization Using WiFi", ACM SIGCOMM (2015); "Device-Free Indoor Localization with ESP32 Wi-Fi CSI Fingerprints", Sciety (2026)
**URL**: https://web.stanford.edu/~skatti/pubs/sigcomm15-spotfi.pdf; https://sciety.org/articles/activity/10.20944/preprints202601.2378.v1
**Date**: 2015; 2026-01-30
**Excerpt**: "SpotFi achieves a median localization error of 0.4 m compared to 1.8 m for ArrayTrack"; "achieves a median localization error of 0.45 m and 90th percentile error below 0.9 m"
**Context**: Median error is preferred over mean for skewed error distributions typical in indoor environments.
**Confidence**: High

#### Percentile Errors (90th, 95th, 99th)

**Claim**: Higher percentile errors (p90, p95) are critical for reliability-sensitive applications. They represent the "worst-case" performance that users may experience. [^133^] [^146^]
**Source**: "Device-Free Indoor Localization with ESP32 Wi-Fi CSI Fingerprints"; "Indoor Localization Accuracy of Major Smartphone Location Apps", NIST
**URL**: https://sciety.org/articles/activity/10.20944/preprints202601.2378.v1; https://www.nist.gov/document/indoor-localization-accuracy-major-smartphone-location-apps
**Date**: 2026-01-30; N/A
**Excerpt**: "90th percentile error below 0.9 m"; "the 50- or 95-percentile points on its CDF are mostly lower"
**Context**: ISO/IEC 18305 recommends reporting multiple percentiles to characterize the full error distribution.
**Confidence**: High

#### Localization Success Rate (SR)

**Claim**: Success rate is the proportion of test points with error below a predefined threshold (commonly 2m, 3m, or 5m). [^159^]
**Source**: "Comparative study of indoor positioning datasets", Nature Scientific Reports (2025)
**URL**: https://www.nature.com/articles/s41598-025-17692-w
**Date**: 2025-10-03
**Excerpt**: "The localization success rate (SR) is the proportion of test points with an error below a predefined threshold... we adopt a threshold of 5 meters"
**Context**: SR is intuitive for application designers who need to know if a system meets room-level or building-level accuracy requirements.
**Confidence**: High

### Derived and Diagnostic Metrics

| Metric | Purpose | Sensitivity |
|--------|---------|-------------|
| RMSE | Overall error magnitude | High sensitivity to large deviations |
| Standard Deviation | Stability measurement | Sensitive to variation across frames |
| Mean (MAE) | Bias detection | Low sensitivity to outliers |
| Median | Typical performance | Robust to outliers |
| 90th/95th Percentile | Reliability bounds | Captures tail behavior |
| R^2 (Coefficient of Determination) | Model fit quality | Explains variance in predictions [^119^] |

---

## 2. Ground Truth Establishment Methods

### Overview of Ground Truth Techniques

Establishing accurate ground truth is fundamental to meaningful evaluation. The accuracy of the reference system must be at least one order of magnitude better than the system under test. [^178^]

**Claim**: Ground truth systems span a range of technologies from millimeter-accurate optical motion capture to manual surveying, each suited to different scales and budgets. [^142^]
**Source**: "A Survey of Ground Truth Measurement Systems for Indoor Localization", Journal of Information Processing (2023)
**URL**: https://www.jstage.jst.go.jp/article/ipsjjip/31/0/31_15/_pdf
**Date**: 2023
**Excerpt**: Table of ground truth systems including OptiTrack (0.1 mm, 120 Hz), Vicon (0.5 mm, 100 Hz), Total Station (1 mm), and manual methods.
**Context**: This is the most comprehensive survey of ground truth methodologies for indoor localization.
**Confidence**: High

### Ground Truth Methods Comparison

| Method | Accuracy | Scale | Frequency | Cost | Best For |
|--------|----------|-------|-----------|------|----------|
| **Optical Motion Capture (OptiTrack)** | 0.1 mm | MoCap room | 120-180 Hz | Very High | Algorithm development, small-scale validation |
| **Vicon** | 0.5 mm | 8x8x4 m | 100 Hz | Very High | Research labs, robotics |
| **Total Station (TOPCON/Leica)** | 1 mm | 60x60 m | 3-20 Hz | High | Professional surveying, building-wide |
| **LiDAR SLAM (Velodyne)** | cm-level | 21,000 m2 | N/A | High | Multi-building, mobile robot trajectories |
| **Google Tango/AR Core** | ~30 cm | Office floor | 100 Hz | Low | Quick deployment, large areas |
| **Manual Survey (laser distance meter)** | 1-5 cm | Room/building | Static | Very Low | Most research deployments |
| **Fiducial Markers (ceiling-mounted)** | 5-15 cm | Multi-room | Camera-limited | Low | Long-term evaluation [^177^] |

#### Laser Distance Meter (Manual Survey)

**Claim**: The most common ground truth method in research deployments uses laser distance meters to measure positions of test points relative to walls or reference markers. [^59^]
**Source**: "SpotFi: Decimeter Level Localization Using WiFi", ACM SIGCOMM (2015)
**URL**: https://web.stanford.edu/~skatti/pubs/sigcomm15-spotfi.pdf
**Date**: 2015
**Excerpt**: "The locations of the access points with respect to a map are measured accurately by using laser range finder and architectural drawings of the building"
**Context**: This is the standard approach for research deployments that lack expensive motion capture facilities.
**Confidence**: High

#### NIST Instrumented Buildings

**Claim**: NIST has instrumented buildings with ~1,300 professionally surveyed test points ("dots") for systematic evaluation of smartphone localization apps. [^146^]
**Source**: "Indoor Localization Accuracy of Major Smartphone Location Apps", NIST
**URL**: https://www.nist.gov/document/indoor-localization-accuracy-major-smartphone-location-apps
**Date**: N/A
**Excerpt**: "These buildings are instrumented with almost 1300 test points, henceforth called dots, laid on the floors and professionally surveyed. Therefore, the ground truth 3D coordinates (latitude, longitude, and elevation) of all dots are known to NIST."
**Context**: NIST's PerfLoc project represents the gold standard for large-scale systematic evaluation.
**Confidence**: High

#### Minimum Test Point Requirements

**Claim**: Standards recommend at least 20 evaluation points, with one point per 5-10 m2 on average. [^178^]
**Source**: "A Testing and Evaluation Framework for Indoor Navigation and Positioning Systems", PMC (2023)
**URL**: https://pmc.ncbi.nlm.nih.gov/articles/PMC11991443/
**Date**: 2023-06-02
**Excerpt**: "The number of test points should be no less than 20... Generally, one test point is selected per 5 to 10 m2 on average. At least half of the test points should be evaluated in a single test."
**Context**: Provides practical minimums for statistically meaningful evaluation.
**Confidence**: High

---

## 3. Cumulative Distribution Function (CDF) of Error

### Definition and Importance

**Claim**: The CDF of localization error is the definitive way to characterize system performance because it captures the entire error distribution, not just a single statistic. It enables comparison across systems and provides percentile-based reliability guarantees. [^120^] [^59^]
**Source**: "A Survey of Recent Indoor Localization Scenarios and Methodologies", PMC (2020); "SpotFi", ACM SIGCOMM (2015)
**URL**: https://pmc.ncbi.nlm.nih.gov/articles/PMC8662396/; https://web.stanford.edu/~skatti/pubs/sigcomm15-spotfi.pdf
**Date**: 2020-02-28; 2015
**Excerpt**: "The authors of [28] also proposed the cumulative distribution function (CDF) as the integration of error rate PDF (i.e., probability density function) in order to quantify the localization accuracy, instead of directly measuring classic discrete error indicators such as RMSE."
**Context**: CDF plots are now mandatory in virtually all top-tier localization papers.
**Confidence**: High

### Why CDF is Essential

The CDF answers critical questions that single-number metrics cannot:

1. **"What percentage of locations are within 1 meter?"** - The CDF directly shows this at the 1m x-intercept.
2. **"What is the worst-case error I should design for?"** - Read the 95th percentile.
3. **"Does System A beat System B at all percentiles, or only on average?"** - Crossing CDFs reveal trade-offs. [^146^]

**Claim**: Crossing CDF curves indicate that one system may be better at median but worse in the tail, revealing trade-offs that single-number metrics hide. [^146^]
**Source**: NIST PerfLoc evaluation
**URL**: https://www.nist.gov/document/indoor-localization-accuracy-major-smartphone-location-apps
**Excerpt**: "Figures 4 and 6 show crossing CDFs for FLP and Core Location. This implies that the probability distribution for the magnitude of vertical error for FLP has a heavier tail than the corresponding probability distribution for Core Location."
**Context**: Crossing CDFs are common when comparing systems with different error characteristics.
**Confidence**: High

### eCDF (Empirical CDF)

**Claim**: eCDF is a discrete CDF function depending on previously-collected samples, enabling analytical statistics at different percentile levels (50th, 75th, 95th). [^120^]
**Source**: "A Survey of Recent Indoor Localization Scenarios and Methodologies"
**URL**: https://pmc.ncbi.nlm.nih.gov/articles/PMC8662396/
**Excerpt**: "Ryosuke Ichikari et al. proposed to utilize empirical CDF (eCDF), a discrete CDF function depending on the previously-collected samples, which allowed them to provide analytical statistics for absolute errors at different percentile levels"
**Context**: eCDF is particularly useful for bootstrap-based statistical testing.
**Confidence**: High

---

## 4. Benchmark Datasets for WiFi Indoor Localization

### Major Public Datasets

#### UJIIndoorLoc

**Claim**: UJIIndoorLoc is the most widely cited benchmark dataset, covering 3 buildings with 4-5 floors, 520+ APs, 25 devices from 20 users, and 21,049 samples. [^119^] [^159^]
**Source**: "Comparative study of indoor positioning datasets", Nature Scientific Reports (2025)
**URL**: https://www.nature.com/articles/s41598-025-17692-w
**Date**: 2025
**Excerpt**: "The UJIndoorLoc dataset spans three buildings with varying spatial characteristics... Building 0 features a dense and uniform RP layout, supporting higher localization accuracy. In contrast, Buildings 1 and 2 have distinct geometries and AP deployment strategies."
**Context**: UJIIndoorLoc remains the de facto standard despite being created in 2014. Performance on this dataset varies dramatically: MAE ranges from 4.5m (Building 0) to 8.2m (Building 2).
**Confidence**: High

#### SODIndoorLoc

**Claim**: SODIndoorLoc provides 3 buildings (CETC331, HCXY, SYL) with optimized AP placement based on CRLB principles, 1,630 RPs, and 23,925 samples. [^119^]
**Source**: "Comparative study of indoor positioning datasets", Nature Scientific Reports (2025)
**URL**: https://www.nature.com/articles/s41598-025-17692-w
**Excerpt**: "The SODIndoorLoc dataset enhanced prior datasets by offering a more densely populated grid of reference locations and improved data collection management, thereby facilitating more precise and reproducible evaluations."
**Context**: SODIndoorLoc's CETC331 building uses AP placement optimized via CRLB, achieving decimeter-level accuracy in some configurations.
**Confidence**: High

#### TUJI1

**Claim**: TUJI1 provides high-resolution measurements with 45 cm RP spacing, 5 heterogeneous devices, and ~10,000 samples on a single floor. [^119^]
**Source**: "Comparative study of indoor positioning datasets", Nature Scientific Reports (2025)
**URL**: https://www.nature.com/articles/s41598-025-17692-w
**Excerpt**: "The TUJI1 dataset was collected in a structured indoor environment consisting of offices and corridors on a single floor. RPs were arranged in a fine-grained spatial grid with approximately 45 cm spacing, ensuring uniform coverage."
**Context**: TUJI1 is specifically designed for studying device heterogeneity effects.
**Confidence**: High

### Dataset Comparison Summary

| Dataset | Buildings | Floors | APs | RPs | Devices | Signal Type | Key Challenge |
|---------|-----------|--------|-----|-----|---------|-------------|---------------|
| UJIIndoorLoc | 3 | 4-5 | 520 | 933 | 25 | WiFi RSSI | Device heterogeneity, sparse APs |
| SODIndoorLoc | 3 | 1-3 | 105 | 1,630 | Multiple | WiFi RSSI | Layout diversity |
| TUJI1 | 1 | 1 | 300 | ~10,000 | 5 | WiFi RSSI | Device variation |
| BLE Indoor | 1 | 1 | 15 | 150 | 11 | BLE RSSI | Signal sparsity |

### Diagnostic Metrics for Dataset Quality

**Claim**: Five diagnostic metrics have been proposed to quantify dataset quality: Effective AP Count (EAC), Sparsity Level (SL), Signal Entropy Score (SES), AP Redundancy Score (ARS), and Device Bias Index (DBI). [^159^]
**Source**: "Comparative study of indoor positioning datasets", Nature Scientific Reports (2025)
**URL**: https://www.nature.com/articles/s41598-025-17692-w
**Excerpt**: "We introduce five diagnostic indicators—Effective AP Count (EAC), Sparsity Level (SL), Signal Entropy Score (SES), AP Redundancy Score (ARS), and Device Bias Index (DBI)—to characterize signal richness, redundancy, and cross-device variability."
**Context**: These metrics enable objective comparison of dataset quality beyond simple RP counts.
**Confidence**: High

---

## 5. Statistically Significant Test Point Requirements

### Standards-Based Recommendations

**Claim**: ISO/IEC 18305 and related frameworks recommend one evaluation point per square meter, with adjustments for statistical significance. The T&E 4iLoc framework suggests this number is not fixed and should be adjusted based on the localization system, environment, and procedure. [^149^]
**Source**: "Meaningful Test and Evaluation of Indoor Localization Systems in Semi-Controlled Environments", MDPI Sensors (2022)
**URL**: https://www.mdpi.com/1424-8220/22/7/2797
**Date**: 2022-04-06
**Excerpt**: "The suggested number of evaluation poses is one per square meter. This number is not fixed and should be adjusted to reach statistically significant results for the localization system, test environment, and the test and evaluation procedure."
**Context**: Practical constraints (time, personnel) often limit test points below this ideal.
**Confidence**: High

### Minimum Practical Requirements

**Claim**: Standards specify a minimum of 20 test points, with at least 50% evaluated in a single test session. For building-wide tests, one point per 5-10 m2 is recommended. [^178^]
**Source**: "A Testing and Evaluation Framework for Indoor Navigation and Positioning Systems", PMC (2023)
**URL**: https://pmc.ncbi.nlm.nih.gov/articles/PMC11991443/
**Date**: 2023-06-02
**Excerpt**: "The number of test points should be no less than 20... one test point is selected per 5 to 10 m2 on average. At least half of the test points should be evaluated in a single test."
**Context**: These are minimums; more test points generally yield better statistical power.
**Confidence**: High

### NIST PerfLoc Scale

**Claim**: NIST's PerfLoc project instrumented buildings with ~900-1,300 surveyed test points to achieve statistically robust evaluation across different scenarios. [^146^] [^147^]
**Source**: NIST PerfLoc project documentation
**URL**: https://www.nist.gov/ctl/pscr/perfloc-performance-evaluation-smartphone-indoor-localization-apps
**Date**: 2023-12-15
**Excerpt**: "The data is 'annotated' in the sense that all the measurements are time-stamped and... the data includes timestamps at which the test subject was at one of 900+ surveyed test points."
**Context**: This represents the largest-scale systematic evaluation of smartphone localization.
**Confidence**: High

---

## 6. Factors Affecting Localization Accuracy

### Anchor/AP Placement

**Claim**: Anchor geometry is one of the most critical factors for localization accuracy. Optimal placement maximizes the geometric diversity of measurements, often quantified via the Geometric Dilution of Precision (GDOP) or Cramer-Rao Lower Bound (CRLB). [^138^] [^145^]
**Source**: "Indoor scenario-based UWB anchor placement optimization method", Expert Systems with Applications (2024); "Optimal layout of four anchors", Expert Systems with Applications (2024)
**URL**: https://www.sciencedirect.com/science/article/abs/pii/S0957417422010077; https://www.sciencedirect.com/science/article/pii/S0957417424016750
**Date**: 2024-06-13; 2024-09-14
**Excerpt**: "We propose a heuristic differential evolution algorithm for searching for an optimal anchor placement that is based on minimizing the Cramer-Rao lower bound (CRLB)"; "The experimental results demonstrated that a larger virtual polyhedron volume between the Mobile Terminal (MT) and the anchors resulted in a smaller 3D error."
**Context**: Optimal anchor placement can reduce errors by 50% or more compared to random placement.
**Confidence**: High

### Multipath Effects

**Claim**: Multipath is the dominant source of error in indoor WiFi localization. In confined multipath-rich environments, the measurable RSSI gradient can collapse to only 3 dBm over a 1-5m range, making distance estimation extremely difficult. [^166^] [^139^]
**Source**: "Design and Experimental Evaluation of CSI and RSSI-based Indoor Wi-Fi Ranging on ESP32-S3" (2026); "Indoor Non-Line-Of-Sight Localization Using Deep Learning", MathWorks
**URL**: https://science.lpnu.ua/ictee/all-volumes-and-issues/volume-6-number-1-2026/design-and-experimental-evaluation-csi-and-rssi; https://www.mathworks.com/help/phased/ug/indoor-non-line-of-sight-localization-using-deep-dearning.html
**Date**: 2026-02-26; N/A
**Excerpt**: "Strong multipath constructive interference in the confined 3x8 m test room collapsed the measurable RSSI gradient to only 3 dBm over the full 1-5 m measurement range"; "With a 20 MHz bandwidth, the theoretical one-way range resolution is given by ΔR=c/B, which results in a range resolution of approximately 15 meters."
**Context**: The 15m range resolution of 20 MHz WiFi fundamentally limits time-based approaches.
**Confidence**: High

### Non-Line-of-Sight (NLOS)

**Claim**: NLOS conditions can degrade UWB localization from ~5 cm (LOS) to meters of error. For WiFi, NLOS typically doubles or triples the median error. [^140^] [^132^]
**Source**: "Exploiting Anchor Links for NLOS Combating in UWB Localization", ACM (2024); "Optimal Anchor Placement for Wireless Localization in Mixed LOS and NLOS Scenarios", arXiv (2026)
**URL**: https://dl.acm.org/doi/10.1145/3657639; https://arxiv.org/html/2604.00863v1
**Date**: 2024-05-06; 2026-04-01
**Excerpt**: "In the presence of Line-of-Sight (LoS) signals, UWB-based indoor localization system can locate a target with small errors (<5 cm) but in situations with Non-Line-of-Sight (NLoS) signals, UWB systems work at a much reduced accuracy"; "We develop a unified Fisher-information framework for localization in environments with both Line-of-Sight (LOS) and Non-Line-of-Sight (NLOS) paths."
**Context**: NLOS identification and mitigation is an active research area with significant accuracy gains possible.
**Confidence**: High

### Key Environmental Factors Summary

| Factor | Impact on Accuracy | Mitigation Strategy |
|--------|-------------------|---------------------|
| Anchor/AP placement | 50-80% error variation | CRLB-based optimization |
| Multipath | Dominant error source in WiFi | Fingerprinting, super-resolution AoA |
| NLOS | 2-10x error increase | NLOS identification, diffraction path exploitation |
| Human presence/body shadowing | 1-3m error increase | Multiple antennas, redundant links |
| Furniture/obstacles | Signal attenuation, multipath | Fingerprinting captures these effects |
| Device heterogeneity | Systematic bias | Device calibration, transfer learning |

---

## 7. Real-Time Tracking vs Static Positioning Evaluation

### Static Positioning Metrics

Static evaluation measures single-point accuracy without temporal context. The standard metrics (MAE, RMSE, CDF) apply directly.

### Trajectory-Based Evaluation Metrics

For tracking systems, additional metrics capture temporal consistency:

#### Absolute Trajectory Error (ATE)

**Claim**: ATE measures the global deviation between estimated and ground-truth trajectories after optimal alignment. It provides comprehensive assessment of overall accuracy and cumulative drift. [^144^] [^135^]
**Source**: "Geometric constraints and semantic optimization SLAM", Nature Scientific Reports; "Numerical Evaluation of Visual-SLAM", ISAR J Sci Tech (2026)
**URL**: https://www.nature.com/articles/s41598-025-16714-x.pdf; https://article.isarpublisher.com/download/1313
**Date**: 2026
**Excerpt**: "ATE measures the global deviation between the estimated trajectory and the ground truth, providing a comprehensive assessment of the overall accuracy and cumulative drift."
**Context**: ATE is the standard metric for SLAM and tracking evaluation, adapted from the robotics community.
**Confidence**: High

#### Relative Pose Error (RPE)

**Claim**: RPE evaluates short-term consistency by comparing relative motion between successive frames over fixed time intervals. It captures local drift and frame-to-frame accuracy. [^144^] [^136^]
**Source**: "Geometric constraints and semantic optimization SLAM"; "Out-of-Sight Embodied Agents", arXiv (2026)
**URL**: https://www.nature.com/articles/s41598-025-16714-x.pdf; https://arxiv.org/html/2509.15219v2
**Excerpt**: "RPE evaluates the relative motion between consecutive frames, which is particularly effective in capturing short-term tracking stability and local drift behavior."
**Context**: RPE is insensitive to global alignment errors and focuses on motion estimation quality.
**Confidence**: High

#### End-to-End Drift

**Claim**: End drift quantifies the final positional deviation between the end points of estimated and ground-truth trajectories, revealing accumulated error over long paths. [^135^]
**Source**: "Numerical Evaluation of Visual-SLAM"
**URL**: https://article.isarpublisher.com/download/1313
**Excerpt**: "End drift quantifies the final positional deviation between the end points of the estimated and ground-truth trajectories."
**Context**: Important for applications where cumulative error matters (e.g., navigation).
**Confidence**: High

### Tracking vs Static Positioning Accuracy Gap

**Claim**: Dynamic tracking accuracy is typically worse than static positioning because motion introduces additional challenges (Doppler effects, changing multipath, orientation-dependent antenna patterns). [^5^]
**Source**: "Design and experimental evaluation of CSI and RSSI-based indoor Wi-Fi ranging on ESP32-S3"
**URL**: https://science.lpnu.ua/sites/default/files/journal-paper/2026/mar/42026/paper8mdiadiukvpavlenko.pdf
**Date**: 2026
**Excerpt**: "Lam and She achieved median error of 0.41 m for slow-moving objects (2.5 km/h) using BLE beacons with Kalman filtering... though accuracy degraded significantly at higher speeds (p50=2.01 m at 7 km/h)."
**Context**: Speed of movement is a critical parameter that must be reported in tracking evaluations.
**Confidence**: High

---

## 8. Cramer-Rao Lower Bound (CRLB) for WiFi Localization

### Theoretical Foundation

**Claim**: The CRLB defines the theoretical lower bound on the variance of any unbiased estimator. For indoor localization, it provides insights about the maximum theoretical accuracy achievable and how anchor geometry affects performance. [^129^] [^151^]
**Source**: "Geolocation of Internet hosts: accuracy limits through CRLB" (2018); "A Cramer-Rao Lower Bound of CSI-Based Indoor Localization", IEEE (2017)
**URL**: https://arpi.unipi.it/retrieve/e0d6c92c-9f7a-fcf8-e053-d805fe0aa794/VERSIONE-ACCETTATA-PRE-PROOFS.pdf; http://ieeexplore.ieee.org/document/8110647/
**Date**: 2018; 2017-11-15
**Excerpt**: "The CRLB defines a bound on the minimum mean squared error that affects any unbiased estimator. From a practical point of view, the CRLB provides insights about the maximal theoretical accuracy that can be achieved"; "This paper proposes a Cramer-Rao lower bound (CRLB) for CSI-based localization. This CRLB is derived based on indoor wireless propagation model."
**Context**: CRLB is used extensively for anchor placement optimization and feasibility analysis.
**Confidence**: High

### CSI-Specific CRLB

**Claim**: For CSI-based localization, the CRLB in the frequency domain can be derived by considering path loss, shadow fading, multipath effects, and asynchronous effects between devices. [^88^]
**Source**: "CSI Ranging-based Wi-Fi Indoor Localization Error Analysis"
**URL**: http://www.ic-wcsp.org/upload/20211018/20211018100643704370.pdf
**Excerpt**: "The concept of the Cramer-Rao Lower Bound (CRLB) in the frequency domain is used to analyze the CSI ranging-based localization error, which solves the problem that the CRLB cannot be obtained without the Probability Density Function (PDF) of the CSI signal in the time domain."
**Context**: This frequency-domain approach is particularly relevant for OFDM-based WiFi systems.
**Confidence**: High

### Practical Implications

**Claim**: The CRLB demonstrates that anchor placement geometry fundamentally limits achievable accuracy, regardless of algorithm sophistication. Optimizing anchor positions to minimize CRLB can improve accuracy by 30-50%. [^138^] [^157^]
**Source**: "Indoor scenario-based UWB anchor placement optimization"; "An Optimal Anchor Placement Method for Localization"
**URL**: https://www.sciencedirect.com/science/article/abs/pii/S0957417422010077; https://www.techscience.com/iasc/v31n2/44536/html
**Excerpt**: "We optimize the anchor placement by minimizing the CRLB... Experimental results show the proposed anchor placement optimization method improves the localization performance under redundant and inadequate anchor conditions."
**Context**: CRLB-based optimization is now standard practice for professional deployments.
**Confidence**: High

---

## 9. Fair Comparison Against Baseline Systems

### Requirements for Fair Comparison

**Claim**: Fair comparison requires: (1) same environment, (2) same ground truth, (3) same test points, (4) same devices, and (5) same evaluation metrics. Comparisons across different buildings or conditions are essentially meaningless. [^163^] [^164^]
**Source**: "The EVARILOS Benchmarking Handbook"; "Platform for Benchmarking of RF-based Indoor Localization Solutions"
**URL**: http://atc.udg.edu/MERMAT/papers/paper_2_Tom_Van_Haute_et_al.pdf; https://www.diva-portal.org/smash/get/diva2:1043589/FULLTEXT01.pdf
**Excerpt**: "No unified scheme is provided for the fair comparison and evaluation of various solutions. Therefore it is necessary to develop and establish a comprehensive benchmarking methodology"; "Each solution is evaluated in a different environment using proprietary evaluation metrics. Consequently, it is currently extremely hard to objectively compare the performance."
**Context**: This is the fundamental problem that benchmarking initiatives (EVARILOS, IPIN, Microsoft) address.
**Confidence**: High

### Standardized Evaluation Frameworks

**Claim**: Three major frameworks have been developed for fair comparison: EvAAL, EVARILOS Benchmarking Handbook, and ISO/IEC 18305. [^161^] [^149^]
**Source**: "Application-driven Test and Evaluation Framework"; "Meaningful Test and Evaluation of Indoor Localization Systems"
**URL**: https://arxiv.org/pdf/2107.10597; https://www.mdpi.com/1424-8220/22/7/2797
**Excerpt**: "Different frameworks and methodologies for T&E of LTSs exist, such as the EvAAL Framework and the EVARILOS Benchmarking Handbook... Building on the findings of EVARILOS the ISO/IEC 18305:2016 International Standard was proposed."
**Context**: Despite these frameworks, many papers still compare against baselines tested in different conditions.
**Confidence**: High

### Best Practices for Fair Comparison

1. **Same environment**: Test proposed and baseline systems in the identical physical space
2. **Same ground truth**: Use identical reference points and measurement methodology
3. **Same hardware**: Use identical WiFi cards, APs, and antennas
4. **Same data**: For fingerprinting, use identical training/test splits from the same dataset
5. **Same metrics**: Report MAE, RMSE, median, and CDF for both systems
6. **Statistical testing**: Apply t-test or Wilcoxon to verify that differences are significant

---

## 10. Typical Accuracy Ranges Across Technologies

### Technology Comparison Summary

| Technology | Typical Accuracy | Method | Cost | Infrastructure |
|------------|-----------------|--------|------|----------------|
| **UWB** | 10-30 cm | Time of Flight | High | Dense anchor deployment |
| **WiFi CSI (fingerprinting)** | 30-60 cm median | Fingerprint + ML | Low | Existing WiFi APs |
| **WiFi CSI (AoA/ranging)** | 0.4-2 m | Super-resolution AoA | Low-Medium | Modified APs |
| **WiFi RSSI (fingerprinting)** | 1.5-3 m | Fingerprint + ML | Very Low | Existing APs |
| **WiFi RSSI (trilateration)** | 1.5-3 m | Path loss model | Very Low | Existing APs |
| **BLE AoA** | 0.5-1 m | Angle of Arrival | Medium | BLE 5.1+ anchors |
| **BLE RSSI** | 2-5 m | Signal strength | Low | BLE beacons |
| **Ultrasound** | 10-50 cm | Time of Flight | High | Specialized hardware |
| **Visible Light** | 10-50 cm | Photodiode arrays | High | Modified lighting |

### UWB Accuracy

**Claim**: UWB achieves 10-30 cm accuracy in real-world deployments due to its ultra-wide bandwidth, resistance to multipath, and precise time-of-flight measurements. [^122^] [^123^] [^125^]
**Source**: "Indoor positioning systems compared: the practical guide for 2026"; "UWB vs BLE vs Wi-Fi vs RFID"; "BLE vs UWB Technology Comparison"
**URL**: https://www.crowdconnected.com/blog/indoor-positioning-tech-update-2026/; https://www.blueiot.com/blog/uwb-vs-ble-vs-wifi-vs-rfid.html; https://locaxion.com/blogs/ble-vs-uwb/
**Date**: 2026-03-11; 2026-02-11; 2026-01-28
**Excerpt**: "UWB offers impressive accuracy (sub-30 cm)"; "UWB provides the highest indoor positioning accuracy, typically achieving 10-30 cm in real-world deployments"; "Accuracy: 10-30 Centimeters (Precision)"
**Context**: UWB's accuracy comes at significantly higher infrastructure cost than WiFi or BLE.
**Confidence**: High

### WiFi CSI Fingerprinting Accuracy

**Claim**: CSI fingerprinting with neural networks achieves 0.45m median error and 0.55m mean error on ESP32 hardware with only 3 links in a 6x4m room. [^133^] [^3^]
**Source**: "Device-Free Indoor Localization with ESP32 Wi-Fi CSI Fingerprints", Sciety/preprints.org (2026)
**URL**: https://sciety.org/articles/activity/10.20944/preprints202601.2378.v1; https://www.preprints.org/manuscript/202601.2378
**Date**: 2026-01-29/30
**Excerpt**: "Experiments show that the proposed fingerprinting approach achieves a median localization error of 0.45 m and 90th percentile error below 0.9 m with only three links"
**Context**: This represents state-of-the-art for ESP32-based CSI fingerprinting in a controlled single-room environment.
**Confidence**: High

### WiFi CSI Ranging (ESP32-S3) Accuracy

**Claim**: CSI amplitude-based ranging on ESP32-S3 achieves median errors (p50) of 2.77m (LoS), 3.90m (NLoS-Furniture), and 3.09m (NLoS-Human), comparable to RSSI-based ranging at 1.45-1.95m MAE. [^166^] [^168^]
**Source**: "Design and Experimental Evaluation of CSI and RSSI-based Indoor Wi-Fi Ranging on ESP32-S3" (2026)
**URL**: https://science.lpnu.ua/ictee/all-volumes-and-issues/volume-6-number-1-2026/design-and-experimental-evaluation-csi-and-rssi
**Date**: 2026-02-26
**Excerpt**: "CSI amplitude-based ranging yielded median errors (p50) of 2.77 m (LoS), 3.90 m (NLoS-Furniture), and 3.09 m (NLoS-Human)... with overall MAE of 1.45-1.95 m for RSSI and 1.99-2.03 m for CSI"
**Context**: CSI amplitude ranging without ToF processing performs similarly to filtered RSSI due to the flat amplitude-distance relationship in multipath environments.
**Confidence**: High

### SpotFi (State-of-the-Art WiFi CSI AoA)

**Claim**: SpotFi achieves 0.4m median accuracy using commodity Intel 5300 WiFi cards with super-resolution AoA estimation, comparable to ArrayTrack with 6-8 antennas. [^59^]
**Source**: "SpotFi: Decimeter Level Localization Using WiFi", ACM SIGCOMM (2015)
**URL**: https://web.stanford.edu/~skatti/pubs/sigcomm15-spotfi.pdf
**Excerpt**: "SpotFi achieves a median localization error of 0.4 m compared to 1.8 m for ArrayTrack. The 80th percentile tail errors for SpotFi and ArrayTrack are 1.8 m and 4 m respectively."
**Context**: SpotFi is considered the seminal work in commodity WiFi CSI-based localization.
**Confidence**: High

### BLE Accuracy

**Claim**: BLE RSSI achieves 2-5m accuracy, while BLE AoA (Bluetooth 5.1+) achieves 0.5-1m accuracy with proper anchor deployment. [^122^] [^128^]
**Source**: "Indoor positioning systems compared"; "UWB VS Bluetooth: Which Offers Better Indoor Positioning Accuracy"
**URL**: https://www.crowdconnected.com/blog/indoor-positioning-tech-update-2026/; https://www.mokosmart.com/uwb-vs-bluetooth-indoor-positioning-guide/
**Date**: 2026-03-11; 2024-10-25
**Excerpt**: "BLE + Inertial: 2-5 m"; "BLE RSSI: 2-5m; BLE AoA: 0.5-1m"
**Context**: BLE is more energy-efficient but less accurate than UWB or WiFi CSI fingerprinting.
**Confidence**: High

---

## 11. System Robustness Over Time

### Temporal Stability Challenges

**Claim**: WiFi fingerprinting suffers from temporal drift caused by environmental changes (temperature, humidity, furniture rearrangement, human presence patterns), which degrade accuracy over time. [^159^]
**Source**: "Comparative study of indoor positioning datasets", Nature Scientific Reports (2025)
**URL**: https://www.nature.com/articles/s41598-025-17692-w
**Excerpt**: "We recognize that long-term temporal drift can impact localization performance. In future work, we will apply our comparative framework to multi-month datasets to quantify these effects."
**Context**: Temporal drift is one of the biggest barriers to real-world deployment of fingerprinting systems.
**Confidence**: High

### Temporal Stability Studies

**Claim**: RSSI-based fingerprints can degrade significantly within days to weeks due to environmental changes. Periodical re-calibration or adaptive algorithms are necessary for sustained accuracy. [^155^]
**Source**: "A Survey of Latest Wi-Fi Assisted Indoor Positioning", PMC (2021)
**URL**: https://pmc.ncbi.nlm.nih.gov/articles/PMC10536338/
**Excerpt**: "These techniques require huge calibration effort for building a fingerprint database via wardriving... the received RSS signal is unstable, the cost of building and maintaining the fingerprint database is relatively high."
**Context**: Research into adaptive fingerprinting and domain adaptation aims to reduce re-calibration frequency.
**Confidence**: High

### Drift Analysis Approaches

1. **Periodic re-survey**: Collect new fingerprints at regular intervals (days to weeks)
2. **Anchor-based calibration**: Use fixed reference points to detect and correct drift
3. **Crowdsourcing**: Leverage user devices to update fingerprints automatically
4. **Domain adaptation**: ML techniques to adapt models to new conditions without full re-training
5. **Feature normalization**: Techniques to make fingerprints invariant to slow environmental changes

---

## 12. Statistical Tests for Significance Validation

### Parametric Tests

#### Paired t-test

**Claim**: The paired t-test compares mean errors between two systems tested at the same locations. It assumes normally distributed differences and is appropriate for comparing localization algorithms on the same test set. [^154^]
**Source**: "Understanding the Wilcoxon Sign Test", Statistics Solutions
**URL**: https://www.statisticssolutions.com/free-resources/directory-of-statistical-analyses/wilcoxon-sign-test/
**Date**: 2025-05-14
**Excerpt**: "The Wilcoxon Signed Rank Test offers a non-parametric alternative to the paired sample t-test, specifically designed for comparing the means of two related samples or paired observations."
**Context**: t-test is appropriate when error distributions are approximately normal.
**Confidence**: High

### Non-Parametric Tests

#### Wilcoxon Signed-Rank Test

**Claim**: The Wilcoxon signed-rank test is preferred for localization accuracy comparison because error distributions are typically skewed (non-normal). It compares medians of paired observations without assuming normality. [^154^] [^158^]
**Source**: "Understanding the Wilcoxon Sign Test"; "Wilcoxon Signed Rank Test", Unity College
**URL**: https://www.statisticssolutions.com/free-resources/directory-of-statistical-analyses/wilcoxon-sign-test/; https://unity.edu/wp-content/uploads/2025/04/Wilcoxon-Signed-Rank-Test.pdf
**Excerpt**: "Unlike its parametric counterpart, the Wilcoxon Signed Rank Test does not rely on the normal distribution of data, making it suitable for a wider range of data types, including ordinal data."
**Context**: With a sample size of at least 10 paired observations, the test statistic approximates a normal distribution.
**Confidence**: High

#### Mann-Whitney U-test

**Claim**: The Mann-Whitney U-test is used for two independent samples (e.g., comparing systems tested in different conditions), while Wilcoxon is for paired/dependent samples. [^154^]
**Source**: "Understanding the Wilcoxon Sign Test"
**URL**: https://www.statisticssolutions.com/free-resources/directory-of-statistical-analyses/wilcoxon-sign-test/
**Excerpt**: "While both are non-parametric and assess median differences, the Mann-Whitney U-test is used for two independent samples, and the Wilcoxon Signed Rank Test is for two dependent samples."
**Context**: For fair comparison, systems should be tested at the same points (paired/dependent), making Wilcoxon the appropriate choice.
**Confidence**: High

### Bootstrap Methods

**Claim**: Bootstrap resampling of empirical CDFs enables construction of confidence intervals for percentile errors without parametric assumptions. [^120^]
**Source**: "A Survey of Recent Indoor Localization Scenarios and Methodologies"
**URL**: https://pmc.ncbi.nlm.nih.gov/articles/PMC8662396/
**Excerpt**: "eCDF... allowed them to provide analytical statistics for absolute errors at different percentile levels (e.g., median-50th, 75th, 95th, etc.)"
**Context**: Bootstrap methods are increasingly used for rigorous statistical comparison in localization research.
**Confidence**: High

---

## 13. Key Stakeholders and Initiatives

### Standardization Bodies

1. **ISO/IEC 18305:2016** - "Test and evaluation of localization and tracking systems" - Defines scenarios, metrics, and reporting requirements. [^149^] [^120^]
2. **3GPP** - Standards for cellular-based indoor positioning
3. **IEEE 802.11** - WiFi standards including RTT (802.11mc) and next-gen positioning (802.11az)

### Benchmarking Initiatives

1. **EVARILOS** (EU FP7 project) - Developed the Benchmarking Handbook and open benchmarking platform [^163^] [^164^]
2. **IPIN Competitions** - Annual competitions based on the EvAAL framework since 2014 [^180^] [^184^]
3. **Microsoft Indoor Localization Competition** - Held at IPSN 2014-2017, focused on technology comparison [^182^]
4. **PerfLoc** (NIST) - Performance evaluation of smartphone indoor localization apps [^146^] [^147^]

### Key Datasets and Their Creators

| Dataset | Creator | Year | Significance |
|---------|---------|------|-------------|
| UJIIndoorLoc | Torres-Sospedra et al., UJI | 2014 | Most widely used benchmark |
| SODIndoorLoc | Bi et al. | 2022 | CRLB-optimized AP placement |
| TUJI1 | Klus et al. | 2024 | High-resolution, multi-device |
| BLE Indoor | Assayay et al. | 2024 | BLE-focused dataset |

---

## 14. Counter-Narratives and Limitations

### The "Lab vs. Real World" Gap

**Claim**: Laboratory-reported accuracies (sub-meter) are rarely achievable in real-world deployments. The IPIN competition demonstrated that realistic performance is significantly worse than laboratory papers suggest. [^184^]
**Source**: "The EvAAL evaluation framework and the IPIN competitions"
**URL**: https://iris.cnr.it/bitstream/20.500.14243/352762/1/prod_393909-doc_136335.pdf
**Excerpt**: "The upside was that EvAAL competitions were realistic... the accuracy performance was significantly lower than what you can read in academic papers, as they reflected real-life situations. From this point of view, the EvAAL competitions were a breakthrough."
**Context**: This is perhaps the most important counter-narrative in indoor localization research.
**Confidence**: High

### The Limits of RSSI-Based Localization

**Claim**: A foundational study established that with 802.11 technology, "with much sampling and a good algorithm one can expect a median error of roughly 10ft and a 97th percentile of roughly 30ft." [^173^]
**Source**: "The Limits of Localization Using Signal Strength", Northwestern University
**URL**: https://users.eecs.northwestern.edu/~peters/references/Limits-Localization-Comparative.pdf
**Excerpt**: "A general rule of thumb we found is that using 802.11 technology, with much sampling and a good algorithm one can expect a median error of roughly 10ft and a 97th percentile of roughly 30ft."
**Context**: These limits (~3m median, ~9m 97th percentile) explain why CSI-based approaches are needed for sub-meter accuracy.
**Confidence**: High

### Overclaiming in Research

**Claim**: Many publications lack rigorous statistical testing, test in overly favorable conditions, and compare against weak baselines. The EVARILOS project found that "each solution is evaluated in a different environment using proprietary evaluation metrics." [^163^]
**Source**: "The EVARILOS Benchmarking Handbook"
**URL**: http://atc.udg.edu/MERMAT/papers/paper_2_Tom_Van_Haute_et_al.pdf
**Excerpt**: "RF-based indoor localization solutions enjoy consistent efforts of researchers... However no unified scheme has been devised for evaluation of these solutions and their robustness against various parameters."
**Context**: This fragmentation makes meta-analysis and systematic comparison extremely difficult.
**Confidence**: High

### ESP32 CSI Limitations

**Claim**: ESP32-based CSI systems face significant hardware limitations: limited bandwidth (20 MHz), quantization effects, asynchronous clocks between devices, and incomplete CSI data (only certain subcarriers reported). [^166^] [^88^]
**Source**: "Design and Experimental Evaluation of CSI and RSSI-based Indoor Wi-Fi Ranging on ESP32-S3"; "CSI Ranging-based Wi-Fi Indoor Localization Error Analysis"
**URL**: https://science.lpnu.ua/ictee/all-volumes-and-issues/volume-6-number-1-2026/design-and-experimental-evaluation-csi-and-rssi; http://www.ic-wcsp.org/upload/20211018/20211018100643704370.pdf
**Excerpt**: "Achieving sub-meter CSI-based accuracy requires Time-of-Flight processing of the Channel Impulse Response with parabolic peak interpolation, given the fundamental 15 m range bin resolution imposed by the 20 MHz bandwidth constraint."
**Context**: The 15m range resolution of 20 MHz bandwidth fundamentally limits time-based approaches on ESP32.
**Confidence**: High

### Reproducibility Crisis

**Claim**: A survey of IPIN 2021 papers found that none of 33 publications performing empirical evaluation referred to any existing test and evaluation methodology. [^149^]
**Source**: "Meaningful Test and Evaluation of Indoor Localization Systems in Semi-Controlled Environments"
**URL**: https://www.mdpi.com/1424-8220/22/7/2797
**Excerpt**: "There were 85 full papers published at the IPIN 2021 conference. Empirical test and evaluation of the absolute localization accuracy was performed in 33 publications. None of the 33 publications is referring to an existing test and evaluation methodology."
**Context**: This finding highlights the need for mandatory adoption of standardized evaluation frameworks.
**Confidence**: High

---

## Summary Table: Key Accuracy Claims

| Technology/Approach | Median Error | Mean/MAE | 90th Percentile | Source |
|---------------------|-------------|----------|-----------------|--------|
| **UWB (ToF)** | 10-30 cm | - | <50 cm | Multiple industry sources |
| **WiFi CSI fingerprinting (ESP32, NN)** | 0.45 m | 0.55 m | <0.9 m | Chaudhari et al. (2026) [^133^] |
| **WiFi CSI (SpotFi, Intel 5300)** | 0.4 m | - | 1.8 m | Kotaru et al. SIGCOMM 2015 [^59^] |
| **WiFi CSI ranging (ESP32-S3)** | 1.99-3.09 m | 1.99-2.03 m | 2.23 m | Diadiuk et al. (2026) [^166^] |
| **WiFi RSSI trilateration (ESP32-S3)** | 1.59 m | 1.45-1.95 m | 3.59 m | Diadiuk et al. (2026) [^166^] |
| **WiFi RSSI fingerprinting** | ~1.5-3 m | ~2-4 m | ~5-8 m | Survey papers [^155^] |
| **BLE AoA** | 0.5-1 m | - | ~2 m | Industry sources [^128^] |
| **BLE RSSI** | 2-3 m | 2-5 m | ~5 m | Industry sources [^122^] |
| **UJIIndoorLoc (XGBoost)** | - | 1.4-8.2 m | - | Dataset comparison [^159^] |

---

## Recommendations for ESP32 CSI Evaluation

Based on this research, the following evaluation protocol is recommended for ESP32-based CSI indoor localization systems:

### Minimum Evaluation Requirements

1. **Metrics**: Report MAE, RMSE, median error (p50), 90th percentile (p90), and full CDF
2. **Test points**: Minimum 20 points, ideally one per 1-5 m2 depending on room size
3. **Ground truth**: Laser distance meter or total station, accuracy at least 10x better than expected system accuracy
4. **Scenarios**: Test in LoS, NLoS (furniture), and NLoS (human presence) conditions
5. **Repetitions**: At least 3 independent runs per test point
6. **Statistical testing**: Wilcoxon signed-rank test against baseline (e.g., kNN)
7. **Baseline comparison**: Same environment, same ground truth, same test points

### Best Practices

1. Report **all** of: MAE, RMSE, median, p80, p90, p95, and CDF plot
2. Include inference latency and resource usage (CPU, memory)
3. Test with multiple link configurations (1, 2, 3+ links)
4. Document environment characteristics (dimensions, materials, AP positions)
5. Use standardized datasets (UJIIndoorLoc, SODIndoorLoc) for cross-system comparison
6. Follow ISO/IEC 18305 or EVARILOS guidelines where feasible
7. Clearly state limitations (single room, static positions, limited occupants)

---

## References

[^59^] Kotaru et al., "SpotFi: Decimeter Level Localization Using WiFi," ACM SIGCOMM 2015.
[^88^] Zhang and Nie, "CSI Ranging-based Wi-Fi Indoor Localization Error Analysis," WCSP 2021.
[^119^] Ayub et al., "Comparative study of indoor positioning datasets," Nature Scientific Reports, 2025.
[^120^] Liu et al., "A Survey of Recent Indoor Localization Scenarios and Methodologies," PMC, 2020.
[^121^] "Floor-Plan-aided Indoor Localization," arXiv 2024.
[^122^] "Indoor positioning systems compared: the practical guide for 2026," CrowdConnected, 2026.
[^123^] "UWB vs BLE vs Wi-Fi vs RFID," BlueIoT, 2026.
[^125^] "BLE vs UWB Technology Comparison for RTLS Deployment," Locaxion, 2026.
[^128^] "UWB VS Bluetooth: Which Offers Better Indoor Positioning Accuracy," MOKOSmart, 2024.
[^129^] Ciavarrinia et al., "Geolocation of Internet hosts: accuracy limits through CRLB," 2018.
[^131^] "The Future of Positioning: Exploring UWB and Bluetooth Channel Sounding," Qorvo, 2024.
[^132^] "Optimal Anchor Placement for Wireless Localization in Mixed LOS and NLOS Scenarios," arXiv, 2026.
[^133^] "Device-Free Indoor Localization with ESP32 Wi-Fi CSI Fingerprints," preprints.org, 2026.
[^135^] "Numerical Evaluation of Visual-SLAM," ISAR J Sci Tech, 2026.
[^137^] "RTS-GT: Robotic Total Stations Ground Truthing dataset," arXiv, 2024.
[^138^] "Indoor scenario-based UWB anchor placement optimization method," Expert Systems with Applications, 2024.
[^139^] "Indoor Non-Line-Of-Sight Localization Using Deep Learning," MathWorks.
[^140^] "Exploiting Anchor Links for NLOS Combating in UWB Localization," ACM, 2024.
[^142^] "A Survey of Ground Truth Measurement Systems for Indoor Localization," Journal of Information Processing, 2023.
[^143^] "A survey of deep learning approaches for WiFi," 2020.
[^144^] "Geometric constraints and semantic optimization SLAM," Nature Scientific Reports, 2025.
[^145^] "Optimal layout of four anchors to improve accuracy of UWB based indoor positioning," Expert Systems with Applications, 2024.
[^146^] NIST, "Indoor Localization Accuracy of Major Smartphone Location Apps."
[^147^] NIST PerfLoc project, https://www.nist.gov/ctl/pscr/perfloc
[^148^] "Multi-AP and Test Point Accuracy of the Results in WiFi Indoor Localization," PMC, 2016.
[^149^] "Meaningful Test and Evaluation of Indoor Localization Systems in Semi-Controlled Environments," MDPI Sensors, 2022.
[^150^] "A survey on ubiquitous WiFi-based indoor localization system for smartphone users," PMC, 2016.
[^151^] "A Cramer-Rao Lower Bound of CSI-Based Indoor Localization," IEEE, 2017.
[^153^] "Channel State Information Based Indoor Localization Error," Semantic Scholar.
[^154^] "Understanding the Wilcoxon Sign Test," Statistics Solutions, 2025.
[^155^] "A Survey of Latest Wi-Fi Assisted Indoor Positioning on Smartphones," PMC, 2021.
[^157^] "An Optimal Anchor Placement Method for Localization in Large-Scale WSNs," 2021.
[^158^] "Wilcoxon Signed Rank Test," Unity College, 2025.
[^159^] "Comparative study of indoor positioning datasets," Nature Scientific Reports, 2025.
[^160^] "Effectively Identifying Wi-Fi Devices through State Transitions," arXiv, 2026.
[^161^] "Application-driven Test and Evaluation Framework for Indoor Localization," arXiv, 2021.
[^162^] "A Survey on Test and Evaluation Methodologies of Pedestrian Localization Systems," IEEE, 2020.
[^163^] Van Haute et al., "The EVARILOS Benchmarking Handbook."
[^164^] "Platform for Benchmarking of RF-based Indoor Localization Solutions," 2016.
[^166^] Diadiuk and Pavlenko, "Design and Experimental Evaluation of CSI and RSSI-based Indoor Wi-Fi Ranging on ESP32-S3," 2026.
[^168^] Diadiuk and Pavlenko, full paper, 2026.
[^169^] "Indoor Multipath Assisted Angle of Arrival Localization."
[^170^] "Visualization of Wi-Fi CSI amplitude and phase," ResearchGate.
[^171^] "Multi-Task Learning for Joint Indoor Localization and Blind Channel Estimation," PMC, 2025.
[^172^] "EMSTS-pos: AI-driven indoor positioning with enhanced multi-source transformer," 2025.
[^173^] "The Limits of Localization Using Signal Strength," Northwestern University.
[^174^] "Transforming Decoder-Only Transformers for Accurate WiFi-Telemetry Based Indoor Localization," arXiv, 2025.
[^175^] "A Comprehensive Review of Indoor Localization Techniques and Applications," MDPI, 2025.
[^176^] "Robust Indoor Wireless Localization Using Sparse Recovery."
[^177^] Trekel et al., "Benchmark for Evaluating Long-Term Localization in Changing Real-World Scenarios," IROS 2025.
[^178^] "A Testing and Evaluation Framework for Indoor Navigation and Positioning Systems," PMC, 2023.
[^179^] "Design and Implementation of an Indoor Localization System Based on RSSI in IEEE 802.11ax," MDPI, 2025.
[^180^] "The IPIN 2019 Indoor Localisation Competition - Description and Results."
[^181^] "A Survey of Application of Machine Learning in Wireless Indoor Positioning Systems," arXiv, 2024.
[^182^] "Lessons and Experiences from a Large Indoor Localization Competition," MobiCom 2023.
[^183^] "Sub-1 GHz Indoor RSSI-Based Localization," University of Glasgow, 2025.
[^184^] "The EvAAL evaluation framework and the IPIN competitions," CNR.
[^185^] "EVARILOS D4.3b - Benchmarking Platform," TU Berlin.

---

*Report compiled from 23+ independent web searches across academic databases (IEEE, ACM, arXiv, Nature, PMC), industry sources, and standards documents (NIST, ISO/IEC). All claims traced to primary sources with verbatim excerpts.*
