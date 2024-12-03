#include <Arduino.h>
#include "handControl.h"
#include "moving.h"
void setup() {
    Serial.begin(115200);
    Serial1.begin(115200);
    angleConfig();
    motorConfig();
}

void loop() {
    handInLoop();
}
