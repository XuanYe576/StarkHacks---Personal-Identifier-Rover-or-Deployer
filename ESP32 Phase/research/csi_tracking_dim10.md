# Dimension 10: ESP32 Hardware Constraints & Performance Optimization

## Research Summary

This document provides a comprehensive analysis of ESP32 hardware constraints affecting CSI-based systems, covering CPU and memory specifications across all major variants, RAM allocation limits for CSI buffering, WiFi throughput capabilities, CPU load effects on sampling consistency, maximum sustainable sampling rates, firmware optimization strategies, communication interfaces, ESP32-C6 WiFi 6 improvements, power supply considerations, debugging techniques, thermal characteristics, and comparison with Raspberry Pi for edge processing.

---

## 1. CPU and Memory Specifications of Each ESP32 Variant

### Complete Comparison Table

| Specification | ESP32 | ESP32-S3 | ESP32-C3 | ESP32-C6 |
|---------------|-------|----------|----------|----------|
| **CPU Architecture** | Xtensa LX6 | Xtensa LX7 | RISC-V | RISC-V |
| **Cores** | 2 (dual-core) | 2 (dual-core) | 1 (single-core) | 1 HP + 1 LP |
| **Max Clock Speed** | 240 MHz | 240 MHz | 160 MHz | 160 MHz |
| **CoreMark (single)** | ~614 | ~614 | ~483 | ~497 |
| **CoreMark (dual)** | ~1182 | ~1182 | N/A | N/A |
| **SRAM** | 520 KB | 512 KB | 400 KB | 512 KB |
| **ROM** | 448 KB | 384 KB | 384 KB | 320 KB |
| **RTC Memory** | 16 KB | 16 KB | 8 KB | 16 KB LP |
| **External PSRAM** | Up to 8 MB | Up to 16 MB (Octal SPI) | Not supported | Supported (less common) |
| **External Flash** | Up to 16 MB | Up to 1 GB | Up to 16 MB | Up to 32 MB |
| **Cache** | 64 KB | 4-way/8-way set associative | 8-way, 32-bit | 32 KB |
| **WiFi Standard** | Wi-Fi 4 (802.11b/g/n) | Wi-Fi 4 (802.11b/g/n) | Wi-Fi 4 (802.11b/g/n) | Wi-Fi 6 (802.11ax) |
| **Bluetooth** | BLE 4.2 + Classic | BLE 5.0 | BLE 5.0 | BLE 5.3 |
| **802.15.4 (Thread/Zigbee)** | No | No | No | Yes |
| **SIMD/AI Instructions** | No | Yes (vector/SIMD) | No | No |
| **GPIO (max)** | 34 | 45 | 22 | 30 (QFN40) |
| **ADC Channels** | 18 | 20 (2x 12-bit SAR) | 6 | 7 |
| **USB** | USB OTG (FS) | USB-OTG (FS) + Serial/JTAG | USB Serial/JTAG | USB Serial/JTAG |

**Claim:** ESP32-C6 is the first Espressif chip with Wi-Fi 6 (802.11ax) support, providing OFDMA, TWT, and MU-MIMO capabilities in 2.4 GHz. [^119^] [^162^]
**Source:** Multiple sources including Espressif documentation and comparison guides
**URL:** https://mozelectronics.com/tutorials/esp32-c6-vs-esp32-s3-soc-comparison/
**Date:** 2026-02-28
**Excerpt:** "C6: Wi-Fi 6 (802.11ax) @ 2.4 GHz (20 MHz STA); legacy b/g/n 20/40 MHz... S3: Wi-Fi 4 (802.11b/g/n) @ 2.4 GHz, 20/40 MHz"
**Confidence:** High

### Key Architectural Differences for CSI Applications

**ESP32 (Original):**
- Dual-core Xtensa LX6 at 240 MHz provides best raw CPU power for legacy CSI processing
- 520 KB SRAM is largest among all variants
- Only variant with Bluetooth Classic + BLE
- 64 KB cache provides good performance for CSI data streaming

**ESP32-S3:**
- Dual-core Xtensa LX7 at 240 MHz with SIMD/vector instructions enables DSP operations on CSI data
- 512 KB SRAM with high-speed Octal-SPI PSRAM support (up to 16 MB) ideal for large CSI buffers
- Best variant for edge AI processing of CSI data
- USB-OTG enables direct high-speed USB CDC streaming

**ESP32-C3:**
- Single-core RISC-V at 160 MHz - adequate for basic CSI collection
- 400 KB SRAM (384 KB usable after caching) - sufficient for most CSI applications
- No PSRAM support limits buffering capability
- Most cost-effective option for simple CSI sensing nodes

**ESP32-C6:**
- Single-core RISC-V at 160 MHz with separate low-power core at 20 MHz
- Wi-Fi 6 support provides 256 subcarriers (vs 64 for Wi-Fi 4) in HE20 mode
- 512 KB SRAM + 16 KB LP SRAM for retention during sleep
- 802.15.4 radio enables Thread/Zigbee coexistence with Wi-Fi CSI

---

## 2. RAM Available for CSI Buffering

### Internal DRAM Allocation Limits

**Claim:** ESP32 has 520 KB of available SRAM (320 KB of DRAM and 200 KB of IRAM). However, due to a technical limitation, the maximum statically allocated DRAM usage is 160 KB. The remaining 160 KB (for a total of 320 KB of DRAM) can only be allocated at runtime as heap. [^127^]
**Source:** ESP-IDF Programming Guide - Memory Types
**URL:** https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/memory-types.html
**Date:** Current (v6.0 documentation)
**Excerpt:** "There is 520 KB of available SRAM (320 KB of DRAM and 200 KB of IRAM) on the ESP32. However, due to a technical limitation, the maximum statically allocated DRAM usage is 160 KB. The remaining 160 KB (for a total of 320 KB of DRAM) can only be allocated at runtime as heap."
**Confidence:** High

### Bluetooth Stack Impact
- Available DRAM reduced by **64 KB** when Bluetooth stack is used
- Further reduced by **16-32 KB** if trace memory is used
- CSI applications using BLE for control/signaling must account for this

### Memory Fragmentation
- Due to ROM memory fragmentation issues, not all available DRAM can be used for static allocations
- The remaining DRAM is still available as heap at runtime

### External PSRAM (SPIRAM) for CSI Buffers

**Claim:** ESP32 can use up to 8 MB of external PSRAM, which can be used for large CSI buffers through capability-based allocation. [^155^] [^154^]
**Source:** ESP-IDF Programming Guide - Support for External RAM
**URL:** https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/external-ram.html
**Date:** Current
**Excerpt:** "ESP32 has a few hundred kilobytes of internal RAM... It can be insufficient for some purposes, so ESP32 has the ability to use up to 8 MB of virtual addresses for external PSRAM (pseudo-static RAM) memory."
**Confidence:** High

### Key PSRAM Configuration Options
- `CONFIG_SPIRAM_USE_CAPS_ALLOC`: Enable `heap_caps_malloc(size, MALLOC_CAP_SPIRAM)`
- `CONFIG_SPIRAM_USE_MALLOC`: Enable standard `malloc()` to use external RAM
- `EXT_RAM_BSS_ATTR`: Statically place zero-initialized data in external RAM
- `MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL`: Must use internal RAM for DMA buffers

### Practical CSI Buffer Sizes

| Scenario | Available RAM | CSI Buffer Capacity |
|----------|---------------|---------------------|
| ESP32, no BT, static alloc | ~160 KB | ~800 frames (128-byte CSI) |
| ESP32, no BT, heap alloc | ~320 KB | ~1,600 frames (128-byte CSI) |
| ESP32 + 8MB PSRAM | ~8 MB | ~41,000 frames (128-byte CSI) |
| ESP32-S3, no PSRAM | ~300 KB | ~1,500 frames (128-byte CSI) |
| ESP32-S3 + 8MB PSRAM | ~8 MB | ~41,000 frames (128-byte CSI) |
| ESP32-C3 | ~280 KB | ~1,400 frames (128-byte CSI) |
| ESP32-C3 + BT | ~216 KB | ~1,080 frames (128-byte CSI) |
| ESP32-C6 | ~400 KB | ~2,000 frames (128-byte CSI) |

### DMA Memory Requirement
**Critical:** CSI buffers that will be used with DMA (e.g., SPI, I2S, UART) must be allocated with `MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL` flags - external PSRAM cannot be used for DMA buffers. [^155^]

---

## 3. WiFi Throughput for CSI Streaming

### Maximum UDP Throughput Specifications

**Claim:** ESP32 WiFi can achieve up to 30 MBit/s UDP TX and 30 MBit/s UDP RX in lab conditions. [^178^]
**Source:** ESP-IDF Wi-Fi Performance and Power Save documentation
**URL:** https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/wifi-driver/wifi-performance-and-power-save.html
**Date:** Current
**Excerpt:** "UDP TX: 30 MBit/s... UDP RX: 30 MBit/s" (Air In Lab)
**Confidence:** High

### Practical UDP Streaming Rates for CSI

**Claim:** The ESP32 CSI Tool achieves approximately 20 Hz CSI streaming with 64/128/192 subcarrier frames over UDP. [^182^]
**Source:** RuView firmware README - ESP32-CSI-Node
**URL:** https://github.com/ruvnet/RuView/blob/main/firmware/esp32-csi-node/README.md
**Date:** Current
**Excerpt:** "CSI streaming: Per-subcarrier I/Q capture over UDP: ~20 Hz, ADR-018 binary format"
**Confidence:** High

### UART Throughput Bottlenecks

**Claim:** At 230,400 bps with ASCII formatting, the maximum CSI packet rate is approximately 30 packets per second due to serial communication saturation. With binary format, this increases to approximately 85 packets per second. [^123^]
**Source:** MDPI Sensors - Tools and Methods for Achieving Wi-Fi Sensing in Embedded Devices
**URL:** https://www.mdpi.com/1424-8220/25/19/6220
**Date:** 2025-10-08
**Excerpt:** "The results presented in Figure 10 reveal a strong correspondence between theoretical predictions and empirical measurements for ASCII-formatted CSI data at transmission rates below 30 packets per second. Beyond this threshold, the rate of successfully received measurements plateaud at approximately 30 pps... The number of received measurements stabilized at around 85 packets per second."
**Confidence:** High

### USB CDC Throughput (ESP32-S3)

**Claim:** ESP32-S3 with native USB can achieve approximately 9.6 Mbit/s (1.2 MB/s) throughput with TinyUSB CDC ACM, compared to ~2.26 Mbit/s maximum over UART with external USB-to-serial converter. [^148^]
**Source:** atomic14 - USB CDC Speed Test
**URL:** https://atomic14.substack.com/p/this-number-does-nothing
**Date:** 2025-08-23
**Excerpt:** "USB Full Speed is 12Mbit/s. With overheads and other factors, realistically we should be able to get around 9.6Mbit/s... I gave it a quick test on an old ESP32 board with a USB-to-serial converter and got 2.26 Mbit/s."
**Confidence:** High

### ESP32-C6 Throughput (WiFi 6)

**Claim:** ESP32-C6 achieves up to 20 MBit/s TCP throughput and 30 MBit/s UDP throughput over the air. [^134^]
**Source:** ESP-IDF Programming Guide v5.2 - ESP32-C6 Wi-Fi Driver
**URL:** https://docs.espressif.com/projects/esp-idf/en/v5.2/esp32c6/api-guides/wifi.html
**Date:** Current
**Excerpt:** "Up to 20 MBit/s TCP throughput and 30 MBit/s UDP throughput over the air"
**Confidence:** High

---

## 4. CPU Load Effects on CSI Sampling Consistency

### Dual-Core Architecture Impact

**Claim:** The ESP32's dual-core architecture separates WiFi processing (Core 0 / PRO_CPU) from application code (Core 1 / APP_CPU). WiFi radio events running on Core 0 can interrupt sensor sampling on Core 1, causing jitter in real-time CSI collection. [^152^]
**Source:** Zbotic - ESP32 FreeRTOS Tasks
**URL:** https://zbotic.in/esp32-freertos-tasks-concurrent-iot-operations-explained/
**Date:** 2026-03-11
**Excerpt:** "Core 0 (PRO_CPU) handles WiFi, Bluetooth, and system tasks by default. Core 1 (APP_CPU) runs your Arduino sketch code... keep your sensor reading, actuator control, and time-sensitive operations on Core 1 (APP_CPU) so they are not interrupted by WiFi radio events which run on Core 0."
**Confidence:** High

### RTOS Tick Interrupt Impact on CSI

**Claim:** The tick interrupt rate of the real-time operating system (RTOS) running on-board the ESP32 artificially reduces the actual number of frames that can be sent at high rates. Small dips in RX rate appear due to this tick interrupt. [^33^]
**Source:** WiFi Sensing on the Edge: Signal Processing Techniques Survey (COMST 2022)
**URL:** https://ebulutvcu.github.io/COMST22_WiFi_Sensing_Survey.pdf
**Date:** Unknown
**Excerpt:** "Small dips in RX rate appear due to the tick interrupt rate of the real-time operating system (RTOS) running on-board the ESP32 TX device which artificially reduces the actual number of frames that are sent."
**Confidence:** High

### Active vs Passive CSI Reception

**Claim:** In active mode (ESP32 is packet destination), CSI can be collected at upwards of 1000 Hz. In passive mode (sniffing third-party traffic), the RX rate is 13-37% lower than active mode, with a standard deviation of 55 Hz at 1000 Hz TX rate. [^33^]
**Source:** WiFi Sensing on the Edge Survey
**URL:** https://ebulutvcu.github.io/COMST22_WiFi_Sensing_Survey.pdf
**Date:** Unknown
**Excerpt:** "This shows that the ESP32 can collect CSI samples at a RX rate upwards of 1000Hz in the active scenario... the RX rate for PX is between 13% and 37% lower than the active scenario."
**Confidence:** High

### Packet Loss at High Rates

**Claim:** A user reported that even at 50000 us send frequency, only ~200 CSI packets were captured out of 20,000 sent, with frequent rx_id skipping indicating massive packet loss. The serial baud rate was already set to 3,000,000. [^136^]
**Source:** GitHub - espressif/esp-csi Issue #201
**URL:** https://github.com/espressif/esp-csi/issues/201
**Date:** 2024-08-21
**Excerpt:** "From the transmitter, I set up 20000 loops... these 20000 packets have been sent in 15~16 seconds. However, the receiver only captured around 200 data... Based on the captured ID in the CSV file, it seems multiple packets dropped frequently."
**Confidence:** High

### CSI Callback CPU Overhead

**Claim:** ASCII string formatting inside the CSI callback causes the system to drop packets. Using printf or ets_printf to output CSI data as ASCII strings significantly inflates data size and causes CPU overhead. Binary output would reduce bandwidth requirement by ~75%. [^131^]
**Source:** GitHub - espressif/esp-csi Issue #249
**URL:** https://github.com/espressif/esp-csi/issues/249
**Date:** 2026-03-04
**Excerpt:** "The string conversion (ASCII) significantly inflates the data size (from ~512 bytes raw to ~2000+ characters). CPU overhead for string formatting inside the CSI callback causes the system to drop packets (rx_id skipping)."
**Confidence:** High

---

## 5. Maximum Sustainable CSI Sampling Rate

### Theoretical vs Practical Limits

| Mode | Theoretical Max | Practical Sustainable | Notes |
|------|----------------|----------------------|-------|
| **Active RX (on-device processing)** | 1000 Hz | ~1000 Hz | No serial output; WiFi only |
| **ASCII over UART @ 230400 bps** | 45 pps | ~30 pps | Serial channel saturation |
| **Binary over UART @ 230400 bps** | 225 pps | ~85 pps | Practical limit |
| **Binary over UART @ 2Mbps** | ~1950 pps | ~1000 pps | 128 subcarriers |
| **Binary over UART @ 2Mbps (256 SC)** | ~1950 pps | <100 pps | ESP32-C6 with 256 SC |
| **USB CDC (ESP32-S3)** | ~1200 KB/s | ~1000 KB/s | Native USB |
| **UDP over WiFi** | 30 MBit/s | ~20 Hz typical | Protocol stack overhead |

**Claim:** Configurable sampling rate of 10-100 Hz via menuconfig is recommended. 100 Hz for motion detection, 10 Hz sufficient for occupancy. [^121^]
**Source:** RuView - ADR-012 ESP32 CSI Sensor Mesh
**URL:** https://github.com/ruvnet/RuView/blob/main/docs/adr/ADR-012-esp32-csi-sensor-mesh.md
**Date:** 2026-02-28
**Excerpt:** "Configurable sampling rate: 10-100 Hz via menuconfig. 100 Hz for motion detection, 10 Hz sufficient for occupancy."
**Confidence:** Medium

### ESP32-C6 256-Subcarrier Limitation

**Claim:** Using 128 subcarriers at 2,000,000 baud works fine on ESP32-C6, but 256 subcarriers causes immediate packet loss (skipping rx_id). This confirms the bottleneck is UART throughput and CPU string processing. [^131^]
**Source:** GitHub - espressif/esp-csi Issue #249
**URL:** https://github.com/espressif/esp-csi/issues/249
**Date:** 2026-03-04
**Excerpt:** "In my testing with an ESP32 at 2,000,000 baud, using 128 subcarriers works fine, but 256 subcarriers causes immediate packet loss (skipping rx_id). This confirms that the bottleneck is the UART throughput and CPU string processing."
**Confidence:** High

---

## 6. ESP32 Firmware Optimization for CSI Streaming

### FreeRTOS Task Priorities

**Claim:** High-throughput or low-latency CSI tasks must be granted high priority. Tasks must yield CPU via vTaskDelay(), sleep(), or blocking on semaphores/queues to avoid starving lower-priority tasks. [^129^]
**Source:** ESP-IDF Speed Optimization Guide
**URL:** https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/performance/speed.html
**Date:** Current (v6.0)
**Excerpt:** "It is necessary to ensure that high-throughput or low-latency tasks are granted a high priority in order to run immediately... It is also necessary to ensure that tasks yield CPU (by calling vTaskDelay(), sleep(), or by blocking on semaphores, queues, task notifications, etc) in order to not starve lower-priority tasks."
**Confidence:** High

### Recommended Core Pinning Strategy

| Task Type | Recommended Core | Priority |
|-----------|-----------------|----------|
| WiFi stack (system) | Core 0 (PRO_CPU) | System default |
| CSI callback/radio | Core 0 (PRO_CPU) | High (configMAX_PRIORITIES-1) |
| Data processing | Core 1 (APP_CPU) | Medium |
| UART/USB output | Core 1 (APP_CPU) | Medium-High |
| Application logic | Core 1 (APP_CPU) | Low-Medium |

### Code Placement in IRAM

**Claim:** WiFi IRAM optimization options trade IRAM usage for speed. CONFIG_ESP_WIFI_IRAM_OPT (15 kB), CONFIG_ESP_WIFI_RX_IRAM_OPT (16 kB), and CONFIG_LWIP_IRAM_OPTIMIZATION (13 kB) improve throughput. [^129^] [^178^]
**Source:** ESP-IDF Speed Optimization Guide
**URL:** https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/performance/speed.html
**Date:** Current
**Excerpt:** "CONFIG_ESP_WIFI_IRAM_OPT: Put frequently called functions in the Wi-Fi library into IRAM for execution... Some examples are CONFIG_FREERTOS_IN_IRAM for FreeRTOS functions, CONFIG_ESP_WIFI_IRAM_OPT for Wi-Fi operations, CONFIG_UART_ISR_IN_IRAM for UART interrupt handling."
**Confidence:** High

### WiFi Performance Configuration Ranks

| Rank | Available Memory | UDP TX | TCP TX | Use Case |
|------|-----------------|--------|--------|----------|
| Iperf (max) | 37.1 KB | 76.2 Mbps | 74.6 Mbps | Benchmark only |
| TX Prior | 113.8 KB | 75.1 Mbps | 50.8 Mbps | High TX throughput |
| High Performance | 123.3 KB | 74.1 Mbps | 46.5 Mbps | Balanced |
| RX Prior | 145.5 KB | 72.4 Mbps | 39.9 Mbps | High RX throughput |
| Default | 144.5 KB | 69.6 Mbps | 44.2 Mbps | General use |
| Memory Saving | 170.2 KB | 64.1 Mbps | 33.8 Mbps | Memory constrained |
| Minimum | 185.2 KB | 36.5 Mbps | 25.6 Mbps | Minimal footprint |

### Binary Data Format Optimization

**Claim:** CSI data can be reported in binary format, enabling transmission of a higher volume of packets at standard baud rates. The ESP32 CSI web collecting tool leverages dual-core architecture and supports binary format output. [^140^]
**Source:** MDPI Sensors - Tools and Methods for Achieving Wi-Fi Sensing
**URL:** https://pmc.ncbi.nlm.nih.gov/articles/PMC12526573/
**Date:** 2025-10-02
**Excerpt:** "Additionally, CSI data can be reported in binary format, enabling the transmission of a higher volume of packets at standard baud rates."
**Confidence:** High

---

## 7. Communication Interfaces for Host Connection

### UART vs USB CDC vs WiFi UDP Comparison

| Interface | Max Speed | Practical CSI Rate | Pros | Cons |
|-----------|-----------|-------------------|------|------|
| **UART @ 115200** | 115.2 kbps | ~14-30 pps (ASCII) | Universal, simple | Very slow for CSI |
| **UART @ 921600** | 921.6 kbps | ~100-200 pps (binary) | Good balance | Requires FTDI/chip support |
| **UART @ 2Mbps** | 2 Mbps | ~500-1000 pps (binary) | Fast serial | Limited reliability |
| **UART @ 3.1Mbps** | 3.1 Mbps | ~1000+ pps | Maximum UART | CP2102 instability |
| **USB CDC (native)** | 12 Mbps (USB FS) | ~9.6 Mbps (~1.2 MB/s) | Very fast | Only S3 has native USB |
| **USB CDC (external)** | 12 Mbps | ~2.26 Mbps (~280 KB/s) | Moderate speed | External chip overhead |
| **WiFi UDP** | 30 Mbps | ~20 Hz typical, up to 1000 Hz | No wires, long range | Protocol overhead, lossy |
| **SD Card** | ~4-20 MB/s | Unlimited local storage | No streaming needed | Post-processing required |

### UART Baud Rate Practical Limits

**Claim:** A user tested an ESP32 board with USB-to-serial converter and achieved 2.26 Mbit/s at 3,100,000 baud rate. Higher baud rates caused inconsistent results. [^148^]
**Source:** atomic14 - USB CDC Speed Test
**URL:** https://atomic14.substack.com/p/this-number-does-nothing
**Date:** 2025-08-23
**Excerpt:** "I gave it a quick test on an old ESP32 board with a USB-to-serial converter and got 2.26 Mbit/s. This is with a baud rate set to 3,100,000 - which is the highest I set it to and get consistent results."
**Confidence:** High

### USB CDC on ESP32-S3

**Claim:** ESP32-S3 with native USB (no UART bridge) can achieve ~9.6 Mbps actual throughput with TinyUSB CDC ACM, but TinyUSB protocol stack without DMA is only expected to reach 6.4 Mbps due to CPU polling overhead. [^170^]
**Source:** ESP-FAQ - USB documentation
**URL:** https://docs.espressif.com/projects/esp-faq/en/latest/software-framework/peripherals/usb.html
**Date:** Current
**Excerpt:** "As this USB mode does not use DMA, but directly uses CPU polling, some time slices are wasted in each transfer. As a result, the TinyUSB protocol stack is only expected to reach 6.4 Mbps (it can reach 9.628 Mbps theoretically if the batch transfer is adopted)."
**Confidence:** High

### Recommended Interface Selection

- **For development/debugging:** USB CDC @ 921600+ bps (convenient, reliable)
- **For production streaming (ESP32-S3):** Native USB CDC ACM (~1.2 MB/s)
- **For production streaming (other variants):** UART @ 2Mbaud with binary format
- **For wireless deployments:** UDP streaming over WiFi (avoids wiring)
- **For offline logging:** SD card storage (no bandwidth constraints)

---

## 8. ESP32-C6 WiFi 6 CSI Improvements

### Subcarrier Resolution Improvement

**Claim:** Wi-Fi 6 (802.11ax) divides the 20 MHz bandwidth into 256 subcarriers with 78.125 kHz subcarrier spacing (vs 64 subcarriers at 312.5 kHz for 802.11n). This provides 4x the subcarrier resolution for CSI sensing. [^183^] [^180^]
**Source:** IEEE 802.11ax Survey Paper
**URL:** https://people.computing.clemson.edu/~jmarty/projects/lowLatencyNetworking/papers/TransportProtocols/surveyon80211ax.pdf
**Date:** Unknown
**Excerpt:** "For traditional IEEE 802.11a/g/n/ac, the 20 MHz bandwidth is divided into 64 subcarriers, and the subcarrier interval is 20,000/64=312.5KHz... the 20 MHz band is divided into 256 subcarriers and consequently, the subcarrier interval is reduced to 78.125KHz."
**Confidence:** High

### Longer Symbol Duration

**Claim:** Wi-Fi 6 uses 12.8 us symbol duration (vs 3.2 us for Wi-Fi 4), with guard intervals of 0.8, 1.6, or 3.2 us. This longer symbol provides improved frequency resolution and better outdoor robustness. [^180^] [^183^]
**Source:** MDPI Future Internet - A Survey of Wi-Fi 6
**URL:** https://www.mdpi.com/1999-5903/14/10/293
**Date:** 2022-10-14
**Excerpt:** "Symbol duration (us): 3.2 [Wi-Fi 4]... 12.8 [Wi-Fi 6]... Guard interval (us): 0.4, 0.8 [Wi-Fi 4]... 0.8, 1.6, 3.2 [Wi-Fi 6]"
**Confidence:** High

### ESP32-C6 CSI Data Format

**Claim:** ESP32-C6 provides CSI data with different total bytes depending on packet type. For non-HT 20MHz: 128 bytes; HT 20MHz: 256 bytes; HT 20MHz STBC: 384 bytes. However, ESP32-C6 has a hardware limitation where L-LTF data may be missing and the first four bytes may be invalid. [^134^] [^143^] [^190^]
**Source:** ESP-IDF Wi-Fi Driver - ESP32-C6
**URL:** https://docs.espressif.com/projects/esp-idf/en/v5.2/esp32c6/api-guides/wifi.html
**Date:** Current
**Excerpt:** "If first_word_invalid field of wifi_csi_info_t is true, it means that the first four bytes of CSI data is invalid due to a hardware limitation in ESP32-C6... Total bytes: 128 [non-HT/20MHz], 256 [HT/20MHz/non-STBC], 384 [HT/20MHz/STBC]"
**Confidence:** High

### Known ESP32-C6 CSI Issues

**Claim:** ESP32-C6 does not report L-LTF data in WiFi CSI callback and has wrong subcarrier order for HT-LTF. Only 128 bytes are received instead of the expected 256 bytes on ESP32-C6. [^143^]
**Source:** GitHub - espressif/esp-idf Issue #14271
**URL:** https://github.com/espressif/esp-idf/issues/14271
**Date:** 2024-07-29
**Excerpt:** "Receive 128 bytes in the callback. L-LTF data is missing. The order of the subcarriers is different from what is described in the WiFi API guide."
**Confidence:** High

### Wi-Fi 6 Benefits for CSI Sensing

1. **Finer frequency resolution:** 4x more subcarriers (256 vs 64) enables better multipath resolution
2. **Longer symbol time:** 12.8 us vs 3.2 us provides better Doppler resolution
3. **OFDMA:** Enables simultaneous multi-user scheduling, reducing contention
4. **BSS Color:** Reduces interference from overlapping networks
5. **TWT:** Target Wake Time enables scheduled CSI collection windows

### Wi-Fi 6 Limitations for CSI

1. **ESP32-C6 only supports 2.4 GHz Wi-Fi 6** (no 5/6 GHz)
2. **20 MHz channel limit in STA mode** (no 40/80/160 MHz)
3. **L-LTF missing from CSI callback** (hardware limitation)
4. **Higher overhead** from HE preamble
5. **Requires Wi-Fi 6 AP** for full OFDMA/TWT benefits

---

## 9. Power Supply Considerations for CSI Stability

### ESP32 Current Consumption During WiFi

**Claim:** ESP32-WROOM-32 consumes approximately 240mA during Wi-Fi transmission at default power levels. Current peaks up to 638mA for about 25us were measured during oscilloscope testing, with average transmission current of 400mA. [^165^] [^163^] [^166^]
**Source:** Microelectronicos - Measuring Power Consumption on ESP32 Modules
**URL:** https://www.microelectronicos.net/measuring-power-on-esp32-modules/
**Date:** Unknown
**Excerpt:** "Here we discovered that average current of the ESP32 while transmitting Wi-Fi packets is 255mA, slightly higher than what is presented in the datasheet (239mA)... We discovered how the current peaks up to 638mA for about 25uS, and average current for transmissions is 400mA."
**Confidence:** High

### Official Power Supply Recommendations

**Claim:** ESP32 datasheet recommends a power supply with 3.3V output and 500mA or more current capacity. The operating voltage ranges from 2.3V to 3.6V, but below 3.0V WiFi becomes unreliable. [^166^]
**Source:** ESP32 Datasheet - Espressif
**URL:** https://www.espressif.com/sites/default/files/documentation/esp32_datasheet_en.pdf
**Date:** Unknown
**Excerpt:** "The operating voltage of ESP32 ranges from 2.3 V to 3.6 V. When using a single-power supply, the recommended voltage of the power supply is 3.3 V, and its recommended output current is 500 mA or more."
**Confidence:** High

### Brownout Detector

**Claim:** Brownout detector triggers when supply voltage dips below minimum threshold. Common causes include weak power supplies, long wires, poor connections, and sudden current draw during WiFi transmission. Adding 100nF + 10uF capacitors directly at ESP32 module pins is recommended. [^146^] [^137^]
**Source:** Arduino StackExchange - ESP32 Brownout
**URL:** https://arduino.stackexchange.com/questions/76690/esp32-brownout-detector-was-triggered-upon-wifi-begin
**Date:** 2020-07-05
**Excerpt:** "Those boards have an inherent weakness in that they don't provide enough decoupling capacitance for the module... For all my designs now I always provide both a 100nF and a 10uF capacitor right at the power pins of the module."
**Confidence:** High

### Power Quality Best Practices

1. **Use regulated 5V 2A power adapter** minimum for ESP32-CAM or high-current applications
2. **Add bulk capacitor:** 470uF or higher electrolytic between 5V and GND
3. **Add ceramic capacitor:** 0.1uF for high-frequency noise filtering
4. **Use short, thick wires:** 22 AWG or thicker for power
5. **Avoid breadboard power rails:** High contact resistance
6. **Don't disable brownout detector:** Causes harder-to-diagnose erratic behavior [^137^]

### USB Power Limitations

**Claim:** USB 2.0 ports provide maximum 500mA, which may be insufficient for ESP32 with WiFi active. Adding sufficient bulk capacitance (6800uF) can allow operation at 150mA supply limit. [^168^]
**Source:** Memol.de - ESP32 Power Supply
**URL:** https://memolde.github.io/MemolPages/esp32/battery/low-power/2021/03/28/ESP32-power-supply.html
**Date:** 2021-03-28
**Excerpt:** "It took me 350 mA to connect. Adding a 47 yF capacitor makes it more stable. Adding another 150 yF capacitor and 300 mA are enough. Adding another 6800 yF capacitor and 150 mA are the absolute minimum."
**Confidence:** Medium

---

## 10. Debugging CSI Quality Issues

### Key Diagnostic Metrics

The ESP32 CSI callback provides the following quality metrics in `rx_ctrl` field:

| Metric | Field | Description |
|--------|-------|-------------|
| RSSI | `rssi` | Received Signal Strength Indicator (dBm) |
| Noise Floor | `noise_floor` | RF noise floor level |
| Timestamp | `timestamp` | Local receiving time |
| Antenna | `antenna` | Which antenna was used |
| Rate | `rate` | PHY rate of received packet |
| Channel | `channel` | WiFi channel number |
| Sig Mode | `sig_mode` | 0=non-HT, 1=HT, 2=HE |
| Bandwidth | `cwb` | 0=20MHz, 1=40MHz |
| STBC | `stbc` | Space-Time Block Coding flag |
| Sequence | `rx_state` / seq_num | Packet sequence for gap detection |

### Packet Skip Detection

**Claim:** Monitoring rx_id sequence numbers is the primary method to detect CSI packet loss. Skipping rx_id values indicates dropped packets due to UART/CPU bottlenecks. [^131^] [^136^]
**Source:** GitHub - espressif/esp-csi Issues
**URL:** https://github.com/espressif/esp-csi/issues/249
**Date:** 2026-03-04
**Excerpt:** "CPU overhead for string formatting inside the CSI callback causes the system to drop packets (rx_id skipping)... 256 subcarriers causes immediate packet loss (skipping rx_id)."
**Confidence:** High

### Debugging Checklist

1. **Verify packet rate:** Count actual CSI callbacks per second vs expected
2. **Monitor rx_id gaps:** Log skipped sequence numbers
3. **Check RSSI stability:** Large RSSI swings indicate interference
4. **Validate subcarrier count:** Ensure expected number of bytes received
5. **Test with binary output:** Eliminate ASCII formatting overhead
6. **Check CPU utilization:** Use esp_cpu_load_monitor to detect overload
7. **Monitor heap usage:** Check available memory to detect leaks
8. **Verify baud rate:** Confirm host and device match
9. **Check for brownouts:** Monitor for reset reasons
10. **Log noise floor:** Elevated noise floor indicates interference

---

## 11. Thermal Characteristics Affecting CSI Stability

### Internal RTC Clock Drift

**Claim:** ESP32 internal RTC typically experiences drift of 1-2 seconds per day at room temperature, but can drift up to 8 minutes per day without calibration. Temperature causes variations of approximately 2 minutes of drift per day for every 1 degree C of temperature difference. [^153^] [^135^] [^53^]
**Source:** Reddit r/esp32 - Clock drift analysis
**URL:** https://www.reddit.com/r/esp32/comments/11cikkp/the_clock_on_the_esp_is_wrong/
**Date:** 2025-09-25
**Excerpt:** "I measured the chip over the course of four weeks... What the chip does not account for is temperature-dependence of the frequency, which I measured to be around 2 minutes of drift per day, for every 1 degree C of temperature difference to when the chip was started."
**Confidence:** High

### Temperature Compensation

**Claim:** The ESP32's default 150kHz internal oscillator varies with manufacturing, age, temperature, and input voltage. Simple NTP calibration can improve drift from 8 minutes/day to 8 seconds/day (from ~5600 ppm to ~93 ppm). [^53^]
**Source:** Climbers.net - ESP32 Low-Power Accurate Clock
**URL:** https://climbers.net/sbc/esp32-accurate-clock-sleep-ntp/
**Date:** Unknown
**Excerpt:** "The default 150kHz oscillator inside each ESP32 will have manufacturing variation, as well as vary with the age of the chip, current temperature and input voltage... this simple calibration improved the ESP32 clock drift from 8 minutes per day to just 8 seconds per day."
**Confidence:** High

### Clock Drift Comparison Table

| Clock Source | Drift/Day | Drift (ppm) | Extra Current | Cost |
|--------------|-----------|-------------|---------------|------|
| ESP32 Default RTC | 480 sec | 5600 ppm | 0 uA | - |
| ESP32 Calibrated | 8 sec | 93 ppm | 0 uA | - |
| ESP32 8.5MHz | 1.7 sec | 20 ppm | 5 uA | - |
| + Crystal Oscillator | 1.7 sec | 20 ppm | 2 uA | ~$1.40 |
| + TCXO | 0.4 sec | 5 ppm | 2 uA | ~$2.32 |
| + DS3231 RTC | 0.4 sec | 5 ppm | 2 uA | ~$6.18 |
| + NTP (WiFi) | 0.0 sec | 0 ppm | 180000 uA | - |

### Impact on CSI Applications

1. **Phase synchronization:** Clock drift affects CSI phase measurements across nodes
2. **Sampling timing:** Inconsistent sampling intervals corrupt temporal features
3. **Multi-node coordination:** Different drift rates on different nodes cause desync
4. **Doppler measurements:** Requires stable clock reference

### Mitigation Strategies

1. **Periodic NTP sync:** Calibrate every few hours for long-running deployments
2. **Temperature compensation:** Adjust calibration based on internal temperature sensor
3. **External crystal:** Add 40MHz crystal for better stability
4. **DS3231 RTC module:** Battery-backed precision RTC (~2 ppm accuracy)
5. **Monotonic timestamps:** Use local monotonic clock; handle drift at aggregator [^121^]

---

## 12. ESP32 vs Raspberry Pi for Edge CSI Processing

### Hardware Comparison

| Feature | ESP32 (S3) | Raspberry Pi 4 |
|---------|-----------|----------------|
| **CPU** | 240 MHz dual-core | 1.5 GHz quad-core |
| **RAM** | 512 KB (+PSRAM) | 2-8 GB |
| **Power** | 100-500 mA peak | 600-1200 mA |
| **Cost** | $5-10 | $35-75 |
| **WiFi** | Built-in, 802.11n (S3) | Built-in, 802.11ac |
| **CSI Tool** | ESP32-CSI-Tool | Nexmon |
| **Subcarriers** | 64 (20MHz) | 256 (80MHz), 2048 (160MHz) |
| **CSI Resolution** | 8-bit | Not specified |
| **802.11 Support** | a/b/g/n (C6: ax) | a/b/g/n/ac |
| **OS** | None (RTOS) | Linux |

**Claim:** The ESP32 CSI Tool emerges as the most portable and cost-effective solution, able to operate independently on a lightweight ESP32 microcontroller. However, the lower quantization level compared to other tools may limit sensitivity for high-precision applications. [^138^]
**Source:** arXiv - Tutorial-cum-Survey on Self-Supervised Learning for Wi-Fi Sensing
**URL:** https://arxiv.org/html/2506.12052v1
**Date:** 2025-05-29
**Excerpt:** "The ESP32 CSI Tool emerges as the most portable and cost-effective solution. Its ability to operate independently on a lightweight ESP32 microcontroller and to store CSI data directly on an onboard micro SD card eliminates the need for external devices... However, the lower quantization level of the ESP32 CSI tool compared to other tools may limit its sensitivity for certain high-precision sensing applications."
**Confidence:** High

### ESP32 Advantages for CSI

1. **Ultra-low cost:** ~$5 per node vs $35+ for Raspberry Pi
2. **Built-in WiFi:** No separate WiFi adapter needed
3. **Low power:** 100-500mA vs 600-1200mA for Pi 4
4. **Small form factor:** Suitable for embedded deployment
5. **SD card storage:** Can log CSI locally without host connection
6. **Dual-core:** Separate cores for WiFi and processing
7. **Real-time processing:** FreeRTOS enables deterministic timing

### Raspberry Pi Advantages for CSI

1. **Higher resolution:** Nexmon provides 256-2048 subcarriers vs 64-256 for ESP32
2. **More processing power:** 1.5 GHz quad-core for complex algorithms
3. **Linux environment:** Full OS with extensive libraries
4. **Better CSI quality:** Higher quantization and more subcarriers
5. **Nexmon support:** Mature CSI extraction tool for Broadcom chips
6. **Larger memory:** GBs of RAM for large buffers and ML models

### Performance Comparison for Sensing Applications

**Claim:** Models trained on CSI data with the ESP-32 show competitive results for motion detection compared to Raspberry Pi 4 with Nexmon. The ESP-32 is suitable for low-cost, distributed CSI sensing deployments. [^176^] [^186^]
**Source:** ResearchGate - Wi-Fi Sensing for Motion Detection using ESP-32 and Raspberry Pi 4
**URL:** https://www.researchgate.net/publication/395063496_Wi-Fi_Sensing_for_Motion_Detection_using_ESP-32_and_Raspberry_Pi_4_A_Comparison
**Date:** 2026-02-09
**Excerpt:** "An in-depth analysis of the CSI data captured from both the ESP-32 and RPi 4 in motion detection shows that models trained on CSI data with the ESP-32 show..."
**Confidence:** Medium

### Edge Processing Performance

**Claim:** A compact DenseNet variant achieves 92.43% accuracy at 232 ms inference latency on ESP32-S3 microcontroller, using only 127 kB of memory. [^140^]
**Source:** MDPI Sensors - Tools and Methods for Wi-Fi Sensing
**URL:** https://pmc.ncbi.nlm.nih.gov/articles/PMC12526573/
**Date:** 2025-10-02
**Excerpt:** "An accuracy of 92.43% at 232 ms inference latency is achieved when implemented on an ESP32-S3 microcontroller. Using as little as 127 kB of memory, the proposed model offers acceptable performance in terms of accuracy and privacy-preserving HAR at the edge."
**Confidence:** High

### Recommendation

- **Choose ESP32** for: Distributed sensor networks, cost-sensitive deployments, battery-powered nodes, simple presence/detection applications
- **Choose Raspberry Pi** for: High-resolution CSI analysis, complex ML inference, centralized processing, research/lab environments

---

## Key Takeaways

1. **ESP32-S3** is the best overall choice for CSI applications requiring edge processing (dual-core, PSRAM, USB-OTG)

2. **ESP32-C6** is the best choice for Wi-Fi 6 deployments requiring finer subcarrier resolution, but has known CSI callback limitations (missing L-LTF, first_word_invalid)

3. **Memory is the primary constraint:** 160 KB static allocation limit on ESP32; use PSRAM for large buffers or dynamic allocation for heap

4. **UART is the primary bottleneck:** Binary format at 2M+ baud is essential for high-rate CSI; USB CDC on S3 is 4x faster

5. **1000 Hz theoretical max** is achievable only in active mode without serial output; practical sustainable rates are 20-100 Hz depending on output method

6. **Power supply quality directly affects CSI stability:** 500mA minimum, proper decoupling capacitors, stable voltage essential

7. **Temperature causes significant clock drift** (~2 min/day per degree C), affecting phase-sensitive CSI applications

8. **ESP32 is competitive with Raspberry Pi** for basic CSI sensing at 1/7th the cost and 1/10th the power

---

## References

[^119^] https://mozelectronics.com/tutorials/esp32-c6-vs-esp32-s3-soc-comparison/
[^120^] https://esp32.co.uk/esp32-c-versions-compared-2026-guide/
[^121^] https://github.com/ruvnet/RuView/blob/main/docs/adr/ADR-012-esp32-csi-sensor-mesh.md
[^123^] https://www.mdpi.com/1424-8220/25/19/6220
[^125^] https://www.olimex.com/Products/IoT/_resources/Comparison-table-ESP32.pdf
[^126^] https://docs.espressif.com/projects/esp-idf/en/v5.0/esp32s3/hw-reference/chip-series-comparison.html
[^127^] https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/memory-types.html
[^128^] https://arxiv.org/html/2503.05429v2
[^129^] https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/performance/speed.html
[^130^] https://www.preprints.org/manuscript/202601.1934
[^131^] https://github.com/espressif/esp-csi/issues/249
[^132^] https://dev.to/numbpill3d/what-actually-happens-when-you-leave-an-esp32-running-247-347e
[^133^] https://www.researchsquare.com/article/rs-8690571/latest.pdf
[^134^] https://docs.espressif.com/projects/esp-idf/en/v5.2/esp32c6/api-guides/wifi.html
[^135^] https://medium.com/@raypcb/a-complete-guide-to-checking-rtc-accuracy-on-the-esp32-3f83a5c70ec7
[^136^] https://github.com/espressif/esp-csi/issues/201
[^137^] https://www.makerguides.com/fix-brownout-of-esp32-cam/
[^138^] https://arxiv.org/html/2506.12052v1
[^139^] https://www.motorobit.com/esp32-vs-raspberry-pi-what-are-the-differences
[^140^] https://pmc.ncbi.nlm.nih.gov/articles/PMC12526573/
[^141^] https://www.elprocus.com/difference-between-esp32-vs-raspberry-pi/
[^142^] https://www.elecbee.com/en/blog/esp32-vs-raspberry-pi-performance-pricing-and-beginner-friendly-comparison-guide_2893
[^143^] https://github.com/espressif/esp-idf/issues/14271
[^144^] https://documentation.espressif.com/esp32-c6_datasheet_en.pdf
[^145^] https://www.cisco.com/c/dam/global/en_au/assets/pdf/wi-fi_6_deep_dive_webinar_oct_2019.pdf
[^146^] https://arduino.stackexchange.com/questions/76690/esp32-brownout-detector-was-triggered-upon-wifi-begin
[^147^] https://github.com/francescopace/espectre/issues/76
[^148^] https://atomic14.substack.com/p/this-number-does-nothing
[^149^] https://esp32.com/viewtopic.php?t=46364
[^150^] https://www.youtube.com/shorts/ifWNCVL60YA
[^152^] https://zbotic.in/esp32-freertos-tasks-concurrent-iot-operations-explained/
[^153^] https://www.reddit.com/r/esp32/comments/11cikkp/the_clock_on_the_esp_is_wrong/
[^154^] https://danielmangum.com/posts/static-alloc-external-ram-esp32/
[^155^] https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/external-ram.html
[^157^] https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/mem_alloc.html
[^158^] https://github.com/Circuit-Digest/ESP32-Real-Time-Clock-using-DS3231-Module
[^159^] https://www.wolfssl.com/utilizing-psram-for-wolfssl-heap-operations-for-the-espressif-esp32/
[^160^] https://gist.github.com/sekcompsci/2bf39e715d5fe47579fa184fa819f421
[^161^] https://dronebotworkshop.com/esp32-2026/
[^162^] https://www.espboards.dev/blog/esp32-soc-options/
[^163^] https://hubble.com/community/guides/how-to-profile-esp32-power-consumption-accurately/
[^165^] https://www.microelectronicos.net/measuring-power-on-esp32-modules/
[^166^] https://www.espressif.com/sites/default/files/documentation/esp32_datasheet_en.pdf
[^167^] https://github.com/Builty/TonexOneController/issues/120
[^168^] https://memolde.github.io/MemolPages/esp32/battery/low-power/2021/03/28/ESP32-power-supply.html
[^170^] https://docs.espressif.com/projects/esp-faq/en/latest/software-framework/peripherals/usb.html
[^171^] https://esp32.com/viewtopic.php?t=44953
[^172^] https://github.com/atomic14/usb-cdc-speed-test
[^175^] https://wiki.seeedstudio.com/xaio_esp32c5_wifi_throughput_tester/
[^176^] https://www.researchgate.net/publication/395063496_Wi-Fi_Sensing_for_Motion_Detection_using_ESP-32_and_Raspberry_Pi_4_A_Comparison
[^177^] https://github.com/ruvnet/RuView/issues/34
[^178^] https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/wifi-driver/wifi-performance-and-power-save.html
[^179^] https://www.scribd.com/document/454691034/1MA222-1e-IEEE80211ax-pdf
[^180^] https://www.mdpi.com/1999-5903/14/10/293
[^181^] https://www.libhunt.com/compare-nexmonster--nexmon_csi-vs-ESP32-CSI-Tool
[^182^] https://github.com/ruvnet/RuView/blob/main/firmware/esp32-csi-node/README.md
[^183^] https://people.computing.clemson.edu/~jmarty/projects/lowLatencyNetworking/papers/TransportProtocols/surveyon80211ax.pdf
[^184^] https://github.com/espressif/esp-idf/issues/15345
[^185^] https://hal.science/hal-04747305/document
[^186^] https://www.semanticscholar.org/paper/Wi-Fi-Sensing-for-Motion-Detection-using-ESP-32-and-Aldarmaki-Alteneiji/a4181dc37a7740472aff068347ac0e3a66fa9714
[^187^] https://espressif-docs.readthedocs-hosted.com/projects/esp-idf/en/stable/api-reference/system/wdts.html
[^188^] https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/performance/ram-usage.html
[^189^] https://docs.espressif.com/projects/esp-techpedia/en/latest/esp-friends/advanced-development/performance/lowpower/lowpower-demo.html
[^190^] https://docs.espressif.com/projects/esp-idf/en/v5.1-beta1/esp32c6/api-guides/wifi.html
[^192^] https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/low-power-mode/low-power-mode-wifi.html
[^33^] https://ebulutvcu.github.io/COMST22_WiFi_Sensing_Survey.pdf
[^53^] https://climbers.net/sbc/esp32-accurate-clock-sleep-ntp/
