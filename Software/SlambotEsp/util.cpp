#include "util.h"
#include <Arduino.h>

void fatal_error(char* message) {
  while(1) {
    Serial.println(message);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
  }
}
