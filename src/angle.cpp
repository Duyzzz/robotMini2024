#include "angle.h"
int currentAngle = 0;
int readAngle(){
    if(Serial3.available()){
        String ang = Serial3.readStringUntil('}');
        // Serial.println(ang);
        currentAngle = ang.toInt();
        return currentAngle;
    }else {
        return currentAngle;
    }
}

void angleConfig(){
    Serial3.begin(115200);
}