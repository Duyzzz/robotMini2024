#include "angle.h"
int currentAngle = 0;
int readAngle(){
    if(Serial2.available()){
        String ang = Serial2.readStringUntil('}');
        currentAngle = ang.toInt();
        return currentAngle;
    }else {
        return currentAngle;
    }
}

void angleConfig(){
    Serial2.begin(115200);
}