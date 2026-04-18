#include <Arduino.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <math.h>
extern "C" {
#include "esp_wifi.h"
}

// Servo 1 controls elevation (tilt), Servo 2 controls azimuth (pan).
constexpr int SERVO_ELEVATION_PIN = 5;
constexpr int SERVO_AZIMUTH_PIN = 6;
constexpr int SERVO_MIN_ANGLE = 0;
constexpr int SERVO_MAX_ANGLE = 180;
constexpr int DEFAULT_ELEVATION = 90;
constexpr int DEFAULT_AZIMUTH = 90;
constexpr int SERIAL_BAUD = 921600;

constexpr int MAX_CSI_BINS = 64;
constexpr uint32_t CSI_STREAM_INTERVAL_MS = 40;

#ifndef WIFI_SSID
#define WIFI_SSID "YOUR_WIFI_SSID"
#endif

#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"
#endif

Servo elevationServo;
Servo azimuthServo;

int elevationDeg = DEFAULT_ELEVATION;
int azimuthDeg = DEFAULT_AZIMUTH;
char commandBuffer[64];
size_t commandLen = 0;

portMUX_TYPE csiMux = portMUX_INITIALIZER_UNLOCKED;

struct CsiFrame {
  uint32_t seq;
  int8_t rssi;
  uint16_t rawLen;
  uint8_t bins;
  uint8_t amplitude[MAX_CSI_BINS];
  int16_t phaseCentiDeg[MAX_CSI_BINS];
};

CsiFrame latestFrame = {};
uint32_t latestFrameVersion = 0;
uint32_t lastStreamedVersion = 0;
uint32_t lastStreamMs = 0;

int clampAngle(int angle) {
  if (angle < SERVO_MIN_ANGLE) {
    return SERVO_MIN_ANGLE;
  }
  if (angle > SERVO_MAX_ANGLE) {
    return SERVO_MAX_ANGLE;
  }
  return angle;
}

void applyServoPositions() {
  elevationServo.write(elevationDeg);
  azimuthServo.write(azimuthDeg);
}

void printStatus() {
  Serial.print("Elevation=");
  Serial.print(elevationDeg);
  Serial.print(" Azimuth=");
  Serial.println(azimuthDeg);
}

void printWifiStatus() {
  Serial.print("WiFi SSID: ");
  Serial.println(WIFI_SSID);
  Serial.print("WiFi status: ");
  Serial.println(WiFi.status() == WL_CONNECTED ? "connected" : "not connected");
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  }
}

void streamCsiFrame(const CsiFrame &frame) {
  Serial.print("CSIv1,");
  Serial.print(frame.seq);
  Serial.print(",");
  Serial.print(frame.rssi);
  Serial.print(",");
  Serial.print(frame.rawLen);
  Serial.print(",");
  Serial.print(frame.bins);
  Serial.print(",A");
  for (int i = 0; i < frame.bins; ++i) {
    Serial.print(",");
    Serial.print(frame.amplitude[i]);
  }
  Serial.print(",P");
  for (int i = 0; i < frame.bins; ++i) {
    Serial.print(",");
    Serial.print(frame.phaseCentiDeg[i]);
  }
  Serial.println();
}

void onCsiReceived(void *ctx, wifi_csi_info_t *info) {
  (void)ctx;
  if (info == nullptr || info->buf == nullptr || info->len < 2) {
    return;
  }

  static uint32_t packetCounter = 0;
  packetCounter++;
  if ((packetCounter & 0x07) != 0) {
    return;
  }

  CsiFrame frame = {};
  frame.seq = packetCounter;
  frame.rssi = info->rx_ctrl.rssi;
  frame.rawLen = info->len;

  int availablePairs = info->len / 2;
  frame.bins = availablePairs > MAX_CSI_BINS ? MAX_CSI_BINS : availablePairs;

  for (int i = 0; i < frame.bins; ++i) {
    int imag = static_cast<int8_t>(info->buf[i * 2]);
    int real = static_cast<int8_t>(info->buf[i * 2 + 1]);

    float amplitude = sqrtf(static_cast<float>(real * real + imag * imag));
    int ampQuantized = static_cast<int>(roundf(amplitude));
    if (ampQuantized < 0) {
      ampQuantized = 0;
    }
    if (ampQuantized > 255) {
      ampQuantized = 255;
    }
    frame.amplitude[i] = static_cast<uint8_t>(ampQuantized);

    float phaseDeg = atan2f(static_cast<float>(imag), static_cast<float>(real)) * 57.2957795f;
    int phaseCentiDeg = static_cast<int>(roundf(phaseDeg * 100.0f));
    if (phaseCentiDeg < -18000) {
      phaseCentiDeg = -18000;
    }
    if (phaseCentiDeg > 18000) {
      phaseCentiDeg = 18000;
    }
    frame.phaseCentiDeg[i] = static_cast<int16_t>(phaseCentiDeg);
  }

  portENTER_CRITICAL(&csiMux);
  latestFrame = frame;
  latestFrameVersion++;
  portEXIT_CRITICAL(&csiMux);
}

void configureCsi() {
  wifi_csi_config_t csiConfig = {};
  csiConfig.lltf_en = true;
  csiConfig.htltf_en = true;
  csiConfig.stbc_htltf2_en = true;
  csiConfig.ltf_merge_en = true;
  csiConfig.channel_filter_en = true;
  csiConfig.manu_scale = false;
  csiConfig.shift = 0;

  esp_err_t err = esp_wifi_set_promiscuous(true);
  if (err != ESP_OK) {
    Serial.printf("esp_wifi_set_promiscuous failed: %d\n", err);
  }

  err = esp_wifi_set_csi_config(&csiConfig);
  if (err != ESP_OK) {
    Serial.printf("esp_wifi_set_csi_config failed: %d\n", err);
  }

  err = esp_wifi_set_csi_rx_cb(&onCsiReceived, nullptr);
  if (err != ESP_OK) {
    Serial.printf("esp_wifi_set_csi_rx_cb failed: %d\n", err);
  }

  err = esp_wifi_set_csi(true);
  if (err != ESP_OK) {
    Serial.printf("esp_wifi_set_csi failed: %d\n", err);
  } else {
    Serial.println("CSI capture enabled.");
  }
}

void connectWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting WiFi");
  uint32_t startMs = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startMs < 15000) {
    Serial.print(".");
    delay(250);
  }
  Serial.println();
  printWifiStatus();
}

void handleSerialCommand(String line) {
  line.trim();
  line.toUpperCase();

  int parsedElevation = elevationDeg;
  int parsedAzimuth = azimuthDeg;
  bool gotAny = false;

  int ePos = line.indexOf('E');
  if (ePos >= 0) {
    parsedElevation = line.substring(ePos + 1).toInt();
    gotAny = true;
  }

  int aPos = line.indexOf('A');
  if (aPos >= 0) {
    parsedAzimuth = line.substring(aPos + 1).toInt();
    gotAny = true;
  }

  if (!gotAny) {
    Serial.println("Use: E<0-180> A<0-180> (e.g. E110 A40)");
    return;
  }

  elevationDeg = clampAngle(parsedElevation);
  azimuthDeg = clampAngle(parsedAzimuth);
  applyServoPositions();
  printStatus();
}

void pollSerialCommands() {
  while (Serial.available()) {
    char c = static_cast<char>(Serial.read());
    if (c == '\n' || c == '\r') {
      if (commandLen > 0) {
        commandBuffer[commandLen] = '\0';
        handleSerialCommand(String(commandBuffer));
        commandLen = 0;
      }
      continue;
    }

    if (commandLen < sizeof(commandBuffer) - 1) {
      commandBuffer[commandLen++] = c;
    }
  }
}

void maybeStreamCsi() {
  if (millis() - lastStreamMs < CSI_STREAM_INTERVAL_MS) {
    return;
  }

  CsiFrame frame = {};
  uint32_t version = 0;
  portENTER_CRITICAL(&csiMux);
  version = latestFrameVersion;
  if (version != lastStreamedVersion) {
    frame = latestFrame;
  }
  portEXIT_CRITICAL(&csiMux);

  if (version == 0 || version == lastStreamedVersion) {
    return;
  }

  streamCsiFrame(frame);
  lastStreamedVersion = version;
  lastStreamMs = millis();
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  elevationServo.setPeriodHertz(50);
  azimuthServo.setPeriodHertz(50);

  elevationServo.attach(SERVO_ELEVATION_PIN, 500, 2400);
  azimuthServo.attach(SERVO_AZIMUTH_PIN, 500, 2400);

  applyServoPositions();

  Serial.println("Two-servo controller ready.");
  Serial.println("Servo 1: elevation (E), Servo 2: azimuth (A)");
  Serial.println("Command format: E<angle> A<angle>");
  printStatus();

  connectWifi();
  configureCsi();
  Serial.println("CSI serial format:");
  Serial.println("CSIv1,<seq>,<rssi>,<raw_len>,<bins>,A,<amp_0>,...,<amp_n>,P,<phase_0>,...,<phase_n>");
}

void loop() {
  pollSerialCommands();
  maybeStreamCsi();
}
