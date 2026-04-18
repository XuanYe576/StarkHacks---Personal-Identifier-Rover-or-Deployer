#ifndef CSI_CONFIG_H
#define CSI_CONFIG_H

// ============================================================================
// WiFi Configuration
// ============================================================================
#ifndef WIFI_SSID_DEF
#define WIFI_SSID_DEF "YOUR_WIFI_SSID"
#endif

#ifndef WIFI_PASSWORD_DEF
#define WIFI_PASSWORD_DEF "YOUR_WIFI_PASSWORD"
#endif

// ============================================================================
// UDP Target Configuration  
// ============================================================================
#ifndef UDP_TARGET_IP_DEF
#define UDP_TARGET_IP_DEF "192.168.1.200"
#endif

#ifndef UDP_TARGET_PORT_DEF
#define UDP_TARGET_PORT_DEF 5005
#endif

// ============================================================================
// CSI Configuration
// ============================================================================
#ifndef CSI_SAMPLE_RATE_HZ
#define CSI_SAMPLE_RATE_HZ 100
#endif

#ifndef CSI_PACKET_RATE_DEF
#define CSI_PACKET_RATE_DEF 10  // 1000 / 100 Hz = 10ms
#endif

// ============================================================================
// Advanced CSI Options
// ============================================================================

// Enable Legacy Long Training Field (L-LTF) CSI extraction
// Recommended: true (required for basic CSI)
#ifndef CSI_LTF_EN
#define CSI_LTF_EN true
#endif

// Enable HT Long Training Field (HT-LTF) CSI extraction
// Recommended: true (provides additional subcarriers)
#ifndef CSI_HT_LTF_EN
#define CSI_HT_LTF_EN true
#endif

// Enable STBC HT-LTF2 CSI extraction
// Recommended: true (provides diversity information)
#ifndef CSI_STBC_EN
#define CSI_STBC_EN true
#endif

// Enable channel filter for noise reduction
// Recommended: false (may remove useful signal components)
#ifndef CSI_CHANNEL_FILTER
#define CSI_CHANNEL_FILTER false
#endif

// ============================================================================
// Buffer Configuration
// ============================================================================

// Maximum CSI buffer size in bytes
// ESP32 provides 384 bytes max (256 subcarriers x 2 bytes I/Q + metadata)
#ifndef CSI_MAX_BUFFER_SIZE
#define CSI_MAX_BUFFER_SIZE 1024
#endif

// Maximum UDP packet size (Ethernet MTU - IP/UDP headers)
// Standard: 1500 - 20 - 8 = 1472 bytes payload
#ifndef UDP_PACKET_MAX_SIZE
#define UDP_PACKET_MAX_SIZE 1472
#endif

// ============================================================================
// Debug Configuration
// ============================================================================

// Enable verbose serial output for debugging
#ifndef DEBUG_VERBOSE
#define DEBUG_VERBOSE 0
#endif

// Enable LED status indicator (GPIO pin number, -1 to disable)
#ifndef STATUS_LED_PIN
#define STATUS_LED_PIN -1
#endif

#endif // CSI_CONFIG_H
