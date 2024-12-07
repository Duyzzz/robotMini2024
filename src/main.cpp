#include <Arduino.h>
#include "handControl.h"
#include "moving.h"
void setup() {
    Serial.begin(115200);
    Serial1.begin(115200);
    handConfig();
}

void loop() {
    // goUp(80, 80);
    // goUp(150, 150);
    // goDown(150, 150);
    // goLeft(150, 150);
    // goRight(150, 150);
    // Serial.println(readAngle());
    readAngle();
    handInLoop();
    // testRulo();
}
