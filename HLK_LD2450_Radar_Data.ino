#include <Adafruit_TinyUSB.h>

// --- USER CONFIGURABLE SETTINGS ---
const float maxDetectionRangeMeters = 6.0; // Max radar range in meters
// ----------------------------------

uint8_t rxBuf[32];
uint8_t rxIdx = 0;

bool previousTargetPresent = false;

void setup() {
  // USB Console output (Computer Serial Monitor)
  Serial.begin(115200);

  unsigned long start = millis();
  while (!Serial && (millis() - start < 5000)) {
    delay(10);
  }

  // HLK-LD2451 Hardware UART connection
  Serial1.begin(115200); 

  Serial.println("\n--- HLK-LD2450 Event-Driven Radar Logger ---");
  Serial.println("Waiting for targets... (Continuous frame logs suppressed)");
}

void parseFrame(uint8_t* frame) {
  uint8_t targetStatus = frame[8];
  uint16_t distanceCm  = frame[9] | (frame[10] << 8);
  int8_t speedVal      = (int8_t)frame[11];

  bool currentTargetPresent = (targetStatus != 0x00 && distanceCm > 0);

  // Trigger ONLY on a brand-new detection event (transition from no target to target)
  if (currentTargetPresent && !previousTargetPresent) {
    float maxRangeCm = maxDetectionRangeMeters * 100.0;

    Serial.println("\n=================================");
    Serial.print("[EVENT] New Target Detected | Status: ");
    switch (targetStatus) {
      case 0x01: Serial.print("Moving"); break;
      case 0x02: Serial.print("Stationary"); break;
      case 0x03: Serial.print("Move & Stat"); break;
      default:   Serial.print("Active"); break;
    }
    Serial.println();

    Serial.print("  > Detection Distance: ");
    Serial.print(distanceCm);
    Serial.println(" cm");
    
    Serial.print("  > Speed: ");
    Serial.print(speedVal);
    Serial.println(" cm/s");

    if (speedVal > 0) {
      // 1. Time elapsed from max range boundary to detection point
      float distanceTraveledCm = maxRangeCm - (float)distanceCm;
      if (distanceTraveledCm > 0) {
        float timeToDetectSec = distanceTraveledCm / (float)speedVal;
        Serial.print("  > Travel Time from Max Range: ");
        Serial.print(timeToDetectSec, 2);
        Serial.println(" seconds");
      } else {
        Serial.println("  > Target detected right at or beyond max range boundary.");
      }

      // 2. Estimated Time of Arrival (ETA) to 0 cm
      float timeToReachSec = (float)distanceCm / (float)speedVal;
      Serial.print("  > Estimated Time to Arrival (ETA): ");
      Serial.print(timeToReachSec, 2);
      Serial.println(" seconds");
    } else {
      Serial.println("  > Target is stationary or moving away (Speed <= 0).");
    }
    Serial.println("=================================\n");
  }

  // Remember state for the next frame
  previousTargetPresent = currentTargetPresent;
}

void loop() {
  while (Serial1.available() > 0) {
    uint8_t b = Serial1.read();

    if (rxIdx < sizeof(rxBuf)) {
      rxBuf[rxIdx++] = b;
    } else {
      rxIdx = 0; // Guard against overflow
    }

    // Check if the 4-byte end tail (F8 F7 F6 F5) has arrived
    if (rxIdx >= 4 &&
        rxBuf[rxIdx - 4] == 0xF8 &&
        rxBuf[rxIdx - 3] == 0xF7 &&
        rxBuf[rxIdx - 2] == 0xF6 &&
        rxBuf[rxIdx - 1] == 0xF5) {

      // Scan buffer for the start header (F4 F3 F2 F1)
      for (int i = 0; i <= rxIdx - 12; i++) {
        if (rxBuf[i]     == 0xF4 &&
            rxBuf[i + 1] == 0xF3 &&
            rxBuf[i + 2] == 0xF2 &&
            rxBuf[i + 3] == 0xF1) {
          
          parseFrame(&rxBuf[i]);
          break;
        }
      }

      rxIdx = 0; // Reset buffer for next frame
    }
  }
}
