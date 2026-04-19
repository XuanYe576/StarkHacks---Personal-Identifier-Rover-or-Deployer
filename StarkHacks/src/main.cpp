#include <Arduino.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <WiFiUdp.h>
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
constexpr uint32_t REMOTE_FRAME_TIMEOUT_MS = 300;

#ifndef DEVICE_HOSTNAME
#define DEVICE_HOSTNAME "tzarium-csi"
#endif

#ifndef WIFI_SSID
#define WIFI_SSID "YOUR_WIFI_SSID"
#endif

#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"
#endif

// WiFi mode:
// 0 = STA (join external AP)
// 1 = AP  (ESP32 creates AP, Mac joins ESP32)
#ifndef CSI_WIFI_AP_MODE
#define CSI_WIFI_AP_MODE 0
#endif

#ifndef WIFI_AP_SSID
#define WIFI_AP_SSID "TzariumCSI"
#endif

#ifndef WIFI_AP_PASSWORD
#define WIFI_AP_PASSWORD "tzarium1234"
#endif

#ifndef WIFI_AP_CHANNEL
#define WIFI_AP_CHANNEL 6
#endif

// Optional UDP CSI stream (full I/Q).
#ifndef CSI_UDP_ENABLE
#define CSI_UDP_ENABLE 0
#endif

#ifndef CSI_UDP_PORT
#define CSI_UDP_PORT 3333
#endif

#ifndef CSI_UDP_TARGET
#define CSI_UDP_TARGET "255.255.255.255"
#endif

#ifndef CSI_UDP_IQ_STREAM
#define CSI_UDP_IQ_STREAM 1
#endif

// Node role:
// 0 = direct (default): send this node CSI to computer target
// 1 = aggregator (A): receive B CSI on relay port, fuse with local CSI, send fused result
// 2 = child (B): send CSI only to aggregator relay target/port
#ifndef CSI_NODE_ROLE
#define CSI_NODE_ROLE 0
#endif

#ifndef CSI_RELAY_LISTEN_PORT
#define CSI_RELAY_LISTEN_PORT 3340
#endif

#ifndef CSI_RELAY_TARGET
#define CSI_RELAY_TARGET "192.168.4.1"
#endif

Servo elevationServo;
Servo azimuthServo;
WiFiUDP csiUdpTx;
WiFiUDP csiUdpRx;
IPAddress csiUdpTargetIp;
uint16_t csiUdpTargetPort = CSI_UDP_PORT;
bool csiUdpReady = false;

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
  int8_t iqI[MAX_CSI_BINS];
  int8_t iqQ[MAX_CSI_BINS];
};

CsiFrame latestFrame = {};
uint32_t latestFrameVersion = 0;
uint32_t lastStreamedVersion = 0;
uint32_t lastStreamMs = 0;

struct RemoteCsiFrame {
  CsiFrame frame;
  bool valid;
  uint32_t receivedMs;
};

RemoteCsiFrame remoteFrame = {};

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
#if CSI_WIFI_AP_MODE
  Serial.print("WiFi mode: AP  SSID: ");
  Serial.println(WIFI_AP_SSID);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());
  Serial.print("Stations connected: ");
  Serial.println(WiFi.softAPgetStationNum());
#else
  Serial.print("WiFi mode: STA  SSID: ");
  Serial.println(WIFI_SSID);
  Serial.print("WiFi status: ");
  Serial.println(WiFi.status() == WL_CONNECTED ? "connected" : "not connected");
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  }
#endif
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

void streamCsiUdpIqFrame(const CsiFrame &frame) {
#if CSI_UDP_ENABLE
  if (!csiUdpReady) {
    return;
  }

  String pkt;
  pkt.reserve(32 + frame.bins * 10);
  pkt += "CSIIQv1,";
  pkt += String(frame.seq);
  pkt += ",";
  pkt += String(frame.rssi);
  pkt += ",";
  pkt += String(frame.rawLen);
  pkt += ",";
  pkt += String(frame.bins);
  pkt += ",I";
  for (int i = 0; i < frame.bins; ++i) {
    pkt += ",";
    pkt += String(frame.iqI[i]);
  }
  pkt += ",Q";
  for (int i = 0; i < frame.bins; ++i) {
    pkt += ",";
    pkt += String(frame.iqQ[i]);
  }

  csiUdpTx.beginPacket(csiUdpTargetIp, csiUdpTargetPort);
  csiUdpTx.write(reinterpret_cast<const uint8_t *>(pkt.c_str()), pkt.length());
  csiUdpTx.endPacket();
#endif
}

bool parseUdpIqPacket(const char *line, CsiFrame &out) {
  if (line == nullptr) {
    return false;
  }
  String s(line);
  s.trim();
  if (!s.startsWith("CSIIQv1,")) {
    return false;
  }

  const int maxParts = 6 + 2 * MAX_CSI_BINS + 8;
  String parts[maxParts];
  int count = 0;
  int start = 0;
  while (start <= s.length() && count < maxParts) {
    int comma = s.indexOf(',', start);
    if (comma < 0) {
      parts[count++] = s.substring(start);
      break;
    }
    parts[count++] = s.substring(start, comma);
    start = comma + 1;
  }
  if (count < 7) {
    return false;
  }

  uint8_t bins = static_cast<uint8_t>(parts[4].toInt());
  if (bins == 0 || bins > MAX_CSI_BINS) {
    return false;
  }
  int markerQ = 6 + bins;
  if (markerQ >= count || parts[5] != "I" || parts[markerQ] != "Q") {
    return false;
  }
  if (count < markerQ + 1 + bins) {
    return false;
  }

  CsiFrame frame = {};
  frame.seq = static_cast<uint32_t>(parts[1].toInt());
  frame.rssi = static_cast<int8_t>(parts[2].toInt());
  frame.rawLen = static_cast<uint16_t>(parts[3].toInt());
  frame.bins = bins;

  for (int i = 0; i < bins; ++i) {
    int8_t iqI = static_cast<int8_t>(parts[6 + i].toInt());
    int8_t iqQ = static_cast<int8_t>(parts[markerQ + 1 + i].toInt());
    frame.iqI[i] = iqI;
    frame.iqQ[i] = iqQ;

    int real = static_cast<int>(iqQ);
    int imag = static_cast<int>(iqI);
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

  out = frame;
  return true;
}

void pollRelayInput() {
#if CSI_UDP_ENABLE && (CSI_NODE_ROLE == 1)
  int packetSize = csiUdpRx.parsePacket();
  while (packetSize > 0) {
    char buf[1400];
    int n = csiUdpRx.read(reinterpret_cast<uint8_t *>(buf), sizeof(buf) - 1);
    if (n > 0) {
      buf[n] = '\0';
      CsiFrame parsed = {};
      if (parseUdpIqPacket(buf, parsed)) {
        remoteFrame.frame = parsed;
        remoteFrame.valid = true;
        remoteFrame.receivedMs = millis();
      }
    }
    packetSize = csiUdpRx.parsePacket();
  }
#endif
}

void fuseWithRemoteIfAvailable(CsiFrame &frame) {
#if CSI_UDP_ENABLE && (CSI_NODE_ROLE == 1)
  if (!remoteFrame.valid) {
    return;
  }
  if ((millis() - remoteFrame.receivedMs) > REMOTE_FRAME_TIMEOUT_MS) {
    return;
  }

  int bins = frame.bins < remoteFrame.frame.bins ? frame.bins : remoteFrame.frame.bins;
  if (bins <= 0) {
    return;
  }

  for (int i = 0; i < bins; ++i) {
    int iAvg = (static_cast<int>(frame.iqI[i]) + static_cast<int>(remoteFrame.frame.iqI[i])) / 2;
    int qAvg = (static_cast<int>(frame.iqQ[i]) + static_cast<int>(remoteFrame.frame.iqQ[i])) / 2;
    frame.iqI[i] = static_cast<int8_t>(iAvg);
    frame.iqQ[i] = static_cast<int8_t>(qAvg);

    float amplitude = sqrtf(static_cast<float>(qAvg * qAvg + iAvg * iAvg));
    int ampQuantized = static_cast<int>(roundf(amplitude));
    if (ampQuantized < 0) {
      ampQuantized = 0;
    }
    if (ampQuantized > 255) {
      ampQuantized = 255;
    }
    frame.amplitude[i] = static_cast<uint8_t>(ampQuantized);

    float phaseDeg = atan2f(static_cast<float>(iAvg), static_cast<float>(qAvg)) * 57.2957795f;
    int phaseCentiDeg = static_cast<int>(roundf(phaseDeg * 100.0f));
    if (phaseCentiDeg < -18000) {
      phaseCentiDeg = -18000;
    }
    if (phaseCentiDeg > 18000) {
      phaseCentiDeg = 18000;
    }
    frame.phaseCentiDeg[i] = static_cast<int16_t>(phaseCentiDeg);
  }
  frame.bins = static_cast<uint8_t>(bins);
  frame.rssi = static_cast<int8_t>((static_cast<int>(frame.rssi) + static_cast<int>(remoteFrame.frame.rssi)) / 2);
#endif
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
    frame.iqI[i] = static_cast<int8_t>(imag);
    frame.iqQ[i] = static_cast<int8_t>(real);

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

void setupUdpStream() {
#if CSI_UDP_ENABLE
  csiUdpReady = false;
  bool wifiReady = false;
#if CSI_WIFI_AP_MODE
  wifiReady = true;
#else
  wifiReady = (WiFi.status() == WL_CONNECTED);
#endif
  if (!wifiReady) {
    Serial.println("UDP stream disabled: WiFi not ready.");
    return;
  }

  const char *targetStr = CSI_UDP_TARGET;
  uint16_t targetPort = CSI_UDP_PORT;
#if CSI_NODE_ROLE == 2
  targetStr = CSI_RELAY_TARGET;
  targetPort = CSI_RELAY_LISTEN_PORT;
#endif
  csiUdpTargetPort = targetPort;

  if (csiUdpTargetIp.fromString(targetStr)) {
    csiUdpReady = true;
  } else {
    IPAddress resolved;
    if (WiFi.hostByName(targetStr, resolved)) {
      csiUdpTargetIp = resolved;
      csiUdpReady = true;
    }
  }

  if (!csiUdpReady) {
    Serial.print("UDP stream disabled: target resolve failed for ");
    Serial.println(targetStr);
    return;
  }

  csiUdpTx.begin(csiUdpTargetPort);
#if CSI_NODE_ROLE == 1
  csiUdpRx.begin(CSI_RELAY_LISTEN_PORT);
#endif
  Serial.print("UDP CSI target: ");
  Serial.print(csiUdpTargetIp);
  Serial.print(":");
  Serial.println(csiUdpTargetPort);
#if CSI_NODE_ROLE == 1
  Serial.print("Aggregator relay listen: ");
  Serial.println(CSI_RELAY_LISTEN_PORT);
#endif
  Serial.println("UDP CSI format: CSIIQv1,<seq>,<rssi>,<raw_len>,<bins>,I,<i...>,Q,<q...>");
#endif
}

void connectWifi() {
#if CSI_WIFI_AP_MODE
  WiFi.mode(WIFI_AP);
  bool ok = WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD, WIFI_AP_CHANNEL);
  Serial.print("Starting AP: ");
  Serial.println(ok ? "ok" : "failed");
  delay(200);
  printWifiStatus();
  setupUdpStream();
#else
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(DEVICE_HOSTNAME);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting WiFi");
  uint32_t startMs = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startMs < 15000) {
    Serial.print(".");
    delay(250);
  }
  Serial.println();
  printWifiStatus();
  setupUdpStream();
#endif
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

  fuseWithRemoteIfAvailable(frame);
  streamCsiFrame(frame);
#if CSI_UDP_ENABLE && CSI_UDP_IQ_STREAM
  streamCsiUdpIqFrame(frame);
#endif
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
#if CSI_UDP_ENABLE
  Serial.println("CSI UDP IQ stream enabled.");
#else
  Serial.println("CSI UDP IQ stream disabled.");
#endif
}

void loop() {
  pollSerialCommands();
  pollRelayInput();
  maybeStreamCsi();
}
