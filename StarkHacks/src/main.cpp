#include <Arduino.h>
#include <ESP32Servo.h>

// Servo 1 controls elevation (tilt), Servo 2 controls azimuth (pan).
constexpr int SERVO_ELEVATION_PIN = 5;
constexpr int SERVO_AZIMUTH_PIN = 6;
constexpr int SERVO_MIN_ANGLE = 0;
constexpr int SERVO_MAX_ANGLE = 180;
constexpr int DEFAULT_ELEVATION = 90;
constexpr int DEFAULT_AZIMUTH = 90;

Servo elevationServo;
Servo azimuthServo;

int elevationDeg = DEFAULT_ELEVATION;
int azimuthDeg = DEFAULT_AZIMUTH;

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

void setup() {
  Serial.begin(115200);
  delay(300);

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
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    handleSerialCommand(line);
  }
}
