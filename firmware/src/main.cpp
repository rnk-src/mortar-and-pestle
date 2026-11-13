#include <Arduino.h>

#include "core/version.h"

void setup() { Serial.begin(115200); }

void loop() {
  Serial.printf("leader protocol v%u\n", static_cast<unsigned>(core::protocol_version()));
  delay(1000);
}
