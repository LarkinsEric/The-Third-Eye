#include <Adafruit_TinyUSB.h>

const uint8_t HEADER[4] = {0xAA, 0xFF, 0x03, 0x00};
const uint8_t TAIL[2]   = {0x55, 0xCC};

const int BUZZER_PIN = 5;          // Connect buzzer to digital pin 12 (or 13)
const int SPEED_THRESHOLD = -30;
const unsigned long buzzerCooldown = 2000;

// Buzzer timing variables (non-blocking)
unsigned long buzzerStartTime = 0;
const unsigned long buzzerDuration = 300; // Duration of beep in milliseconds (0.3 seconds)
bool buzzerIsOn = false;

// State tracking to detect a *new* arrival
bool previousTargetPresent = false;
unsigned long lastBuzzerTriggerTime = 0;

uint8_t rxBuffer[64];
uint8_t rxIndex = 0;

void setup() {
  Serial.begin(115200);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  unsigned long start = millis();
  while (!Serial && (millis() - start < 5000)) {
    delay(10);
  }

  Serial1.begin(115200);

  Serial.println("\n--- HLK-LD2450 Timed Buzzer Alert Active ---");
}

int16_t decodeSigned(uint8_t lo, uint8_t hi) {
  uint16_t raw = lo | (hi << 8);
  int16_t val = raw & 0x7FFF;
  return (hi & 0x80) ? val : -val;
}

bool parseTarget(uint8_t targetNum, uint8_t* data) {
  int16_t x_mm = decodeSigned(data[0], data[1]);
  int16_t y_mm = decodeSigned(data[2], data[3]);
  int16_t speed_cms = decodeSigned(data[4], data[5]);

  if (y_mm == 0 && x_mm == 0) return false;
  if (speed_cms < 0) {
    Serial.print("Target ");
    Serial.print(targetNum);
    Serial.print(" | X: ");
    Serial.print(x_mm);
    Serial.print(" mm | Y (Depth): ");
    Serial.print(y_mm);
    Serial.print(" mm | Speed: ");
    Serial.print(speed_cms);
    Serial.println(" cm/s");
  }

  // Return true if target meets your condition (e.g., speed > 60 cm/s or within a certain distance)
  if (speed_cms < SPEED_THRESHOLD && (y_mm) < 2000) {
    return true;
  }
  return false;
}

void parseFrame(uint8_t* frame, uint8_t length) {
  if (length < 30) return;

  bool currentTargetDetected = false;

  // Check all 3 target slots
  for (uint8_t i = 0; i < 3; i++) {
    if (parseTarget(i + 1, &frame[4 + (i * 8)])) {
      currentTargetDetected = true;
    }
  }

  // Trigger buzzer ONLY on the transition from NO target to TARGET detected
  if (currentTargetDetected && !previousTargetPresent) {
    if (millis() - lastBuzzerTriggerTime >= buzzerCooldown) {
      digitalWrite(BUZZER_PIN, HIGH);
      buzzerStartTime = millis();
      lastBuzzerTriggerTime = millis();
      buzzerIsOn = true;
      Serial.println("--> Alert Triggered: Buzzer On!");
    }
  }

  // Remember state for the next frame
  previousTargetPresent = currentTargetDetected;
}

void loop() {
  // 1. Handle non-blocking buzzer turn-off
  if (buzzerIsOn && (millis() - buzzerStartTime >= buzzerDuration)) {
    digitalWrite(BUZZER_PIN, LOW);
    buzzerIsOn = false;
  }

  // 2. Read incoming radar bytes continuously
  while (Serial1.available() > 0) {
    uint8_t b = Serial1.read();

    if (rxIndex < sizeof(rxBuffer)) {
      rxBuffer[rxIndex++] = b;
    } else {
      rxIndex = 0;
    }

    if (rxIndex >= 30 &&
        rxBuffer[rxIndex - 2] == TAIL[0] &&
        rxBuffer[rxIndex - 1] == TAIL[1]) {

      for (uint8_t i = 0; i <= rxIndex - 30; i++) {
        if (rxBuffer[i]     == HEADER[0] &&
            rxBuffer[i + 1] == HEADER[1] &&
            rxBuffer[i + 2] == HEADER[2] &&
            rxBuffer[i + 3] == HEADER[3]) {

          parseFrame(&rxBuffer[i], rxIndex - i);
          break;
        }
      }
      rxIndex = 0;
    }
  }
}
