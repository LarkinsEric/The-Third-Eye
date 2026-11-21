#include <Adafruit_TinyUSB.h>

const int BUZZER_PIN = 5;            
const unsigned long buzzerDuration = 300; // Duration of beep in ms
unsigned long buzzerStartTime = 0;
bool buzzerIsOn = false;

bool previousTargetPresent = false;

uint8_t rxBuf[64];
uint8_t rxIdx = 0;

void setup() {
  Serial.begin(115200);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  // HLK-LD2451 Hardware UART connection
  Serial1.begin(115200); 

  Serial.println("\n--- HLK-LD2451 Continuous Telemetry & Buzzer Active ---");
}

void parseFrame(uint8_t* frame, int length) {
  bool currentTargetPresent = false;

  // Check if it's a 10-byte "No Target" packet
  if (length == 10 && frame[4] == 0x00 && frame[5] == 0x00) {
    Serial.println("Status: No Target");
    currentTargetPresent = false;
  } 
  else if (length >= 14) {
    // Longer packet containing target data
    uint8_t targetStatus = frame[8];
    uint16_t distanceCm  = frame[9] | (frame[10] << 8);
    int8_t speedVal      = (int8_t)frame[11];

    currentTargetPresent = (targetStatus != 0x00 && distanceCm > 0);

    Serial.print("Status: ");
    switch (targetStatus) {
      case 0x01: Serial.print("Moving     "); break;
      case 0x02: Serial.print("Stationary "); break;
      default:   Serial.print("Active     "); break;
    }
    Serial.print(" | Distance: ");
    Serial.print(distanceCm);
    Serial.print(" cm | Speed: ");
    Serial.print(speedVal);
    Serial.println(" cm/s");
  }

  // Trigger buzzer ONLY on the transition from no target to target detected
  if (currentTargetPresent && !previousTargetPresent) {
    digitalWrite(BUZZER_PIN, HIGH);
    buzzerStartTime = millis();
    buzzerIsOn = true;
    Serial.println("--> [BUZZER TRIGGERED] Target appeared!");
  }

  previousTargetPresent = currentTargetPresent;
}

void loop() {
  // Handle non-blocking buzzer turn-off
  if (buzzerIsOn && (millis() - buzzerStartTime >= buzzerDuration)) {
    digitalWrite(BUZZER_PIN, LOW);
    buzzerIsOn = false;
  }

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
      for (int i = 0; i <= rxIdx - 10; i++) {
        if (rxBuf[i]     == 0xF4 &&
            rxBuf[i + 1] == 0xF3 &&
            rxBuf[i + 2] == 0xF2 &&
            rxBuf[i + 3] == 0xF1) {
          
          int frameLength = rxIdx - i;
          parseFrame(&rxBuf[i], frameLength);
          break;
        }
      }

      rxIdx = 0; // Reset buffer for next frame
    }
  }
}