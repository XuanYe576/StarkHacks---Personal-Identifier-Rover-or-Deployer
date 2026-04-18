/*
 * ESP32 CSI Transmitter Firmware
 * 
 * Streams CSI data via UDP in binary format.
 * Compatible with ESP32, ESP32-S3, ESP32-C6.
 * 
 * Binary frame format:
 *   [magic: 4 bytes uint32 = 0xC5110001]
 *   [timestamp: 8 bytes double (seconds)]
 *   [rssi: 4 bytes int32]
 *   [csi_length: 4 bytes uint32]
 *   [csi_data: N bytes (int8 I/Q pairs)]
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include "config.h"

// ============== CONFIGURATION ==============
const char* WIFI_SSID = WIFI_SSID_DEF;
const char* WIFI_PASS = WIFI_PASSWORD_DEF;
const char* UDP_HOST = UDP_TARGET_IP_DEF;
const int UDP_PORT = UDP_TARGET_PORT_DEF;
const int CSI_PACKET_RATE_MS = CSI_PACKET_RATE_DEF;

// Magic header for frame sync
const uint32_t MAGIC_HEADER = 0xC5110001;

WiFiUDP udp;
uint32_t packet_count = 0;
int64_t start_time_us = 0;
volatile bool csi_enabled = false;

// ============== CSI CALLBACK ==============
void IRAM_ATTR csi_callback(void* ctx, wifi_csi_info_t* data) {
    if (!data || !data->buf) return;
    
    // Prepare binary frame
    uint32_t csi_len = data->len;
    int32_t rssi = (int32_t)data->rx_ctrl.rssi;
    double timestamp = (double)(esp_timer_get_time() - start_time_us) / 1e6;
    
    // Send header
    udp.beginPacket(UDP_HOST, UDP_PORT);
    udp.write((uint8_t*)&MAGIC_HEADER, 4);
    udp.write((uint8_t*)&timestamp, 8);
    udp.write((uint8_t*)&rssi, 4);
    udp.write((uint8_t*)&csi_len, 4);
    
    // Send CSI data (I/Q pairs)
    udp.write(data->buf, csi_len);
    udp.endPacket();
    
    packet_count++;
}

// ============== WIFI EVENT HANDLER ==============
void WiFiEvent(WiFiEvent_t event) {
    switch (event) {
        case ARDUINO_EVENT_WIFI_STA_GOT_IP:
            Serial.println("\nWiFi Connected!");
            Serial.print("IP: ");
            Serial.println(WiFi.localIP());
            Serial.print("Gateway: ");
            Serial.println(WiFi.gatewayIP());
            break;
        case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
            Serial.println("WiFi Disconnected! Reconnecting...");
            csi_enabled = false;
            break;
        default:
            break;
    }
}

// ============== SETUP ==============
void setup() {
    Serial.begin(921600);
    delay(1000);
    
    Serial.println("==================================");
    Serial.println("ESP32 CSI Transmitter");
    Serial.println("Version: 1.0.0");
    Serial.println("==================================");
    
    // Register WiFi event handler
    WiFi.onEvent(WiFiEvent);
    
    // Connect to WiFi
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.persistent(true);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    
    Serial.print("Connecting to ");
    Serial.print(WIFI_SSID);
    
    int retry_count = 0;
    while (WiFi.status() != WL_CONNECTED && retry_count < 60) {
        delay(500);
        Serial.print(".");
        retry_count++;
    }
    
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\nFailed to connect to WiFi! Restarting...");
        delay(2000);
        ESP.restart();
        return;
    }
    
    // Start UDP
    udp.begin(0);
    
    // Record start time
    start_time_us = esp_timer_get_time();
    
    // Configure CSI
    wifi_csi_config_t csi_config = {
        .lltf_en = CSI_LTF_EN,
        .htltf_en = CSI_HT_LTF_EN,
        .stbc_htltf2_en = CSI_STBC_EN,
        .ltf_merge_en = true,
        .channel_filter_en = CSI_CHANNEL_FILTER,
        .manu_scale = false,
        .shift = 0
    };
    
    esp_wifi_set_csi_config(&csi_config);
    esp_wifi_set_csi_rx_cb(csi_callback, NULL);
    esp_wifi_set_csi(true);
    csi_enabled = true;
    
    Serial.println("\nCSI streaming started!");
    Serial.print("Sample Rate: ");
    Serial.print(CSI_SAMPLE_RATE_HZ);
    Serial.println(" Hz");
    Serial.print("Target: ");
    Serial.print(UDP_HOST);
    Serial.print(":");
    Serial.println(UDP_PORT);
    Serial.println("==================================");
}

// ============== MAIN LOOP ==============
void loop() {
    // Send ping to router to trigger CSI
    static uint32_t last_ping = 0;
    uint32_t now = millis();
    
    if (now - last_ping >= CSI_PACKET_RATE_MS) {
        last_ping = now;
        
        // Send a null data frame to generate CSI
        // This creates artificial traffic to trigger CSI collection
        uint8_t ping_data[UDP_PACKET_MAX_SIZE];
        memset(ping_data, 0, sizeof(ping_data));
        
        // Add magic identifier to ping for debugging
        ping_data[0] = 0xDE;
        ping_data[1] = 0xAD;
        ping_data[2] = 0xBE;
        ping_data[3] = 0xEF;
        
        if (WiFi.status() == WL_CONNECTED) {
            udp.beginPacket(WiFi.gatewayIP(), UDP_PORT);
            udp.write(ping_data, 64);
            udp.endPacket();
        }
    }
    
    // Status print every 5 seconds
    static uint32_t last_status = 0;
    if (now - last_status >= 5000) {
        last_status = now;
        
        if (WiFi.status() == WL_CONNECTED) {
            Serial.print("[OK] Packets: ");
            Serial.print(packet_count);
            Serial.print(" | Rate: ");
            Serial.print(packet_count / (now / 1000.0), 1);
            Serial.println(" pkt/s");
        } else {
            Serial.println("[WARN] WiFi disconnected!");
        }
    }
    
    // Watchdog - reconnect if disconnected
    if (WiFi.status() != WL_CONNECTED && csi_enabled) {
        Serial.println("Connection lost. Attempting reconnect...");
        csi_enabled = false;
        esp_wifi_set_csi(false);
        delay(1000);
    }
    
    if (WiFi.status() == WL_CONNECTED && !csi_enabled) {
        Serial.println("Reconnected! Restarting CSI...");
        esp_wifi_set_csi(true);
        csi_enabled = true;
    }
    
    delay(1);
}
