// Toolchain smoke test: proves the espressif32 platform, xtensa-esp32s3
// toolchain and arduino framework are intact without dragging in the core.
// Built by `pio run -e esp32s3_smoke`. Not shipped.
#include <Arduino.h>

void setup() {
  Serial.begin(115200);
}

void loop() {
  Serial.println("smoke");
  delay(1000);
}
