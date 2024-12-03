#include "eyeLine.h"

bool checkEye(uint8_t eye, uint16_t reference){
    if(analogRead(eye) >= reference) return true;
    else return false;
}

int readEyes(){
    int32_t eyeValue = 0;
    for (int8_t i = 0; i < 8; i++)
    {
        eyeValue = eyeValue|checkEye(eyeList[i], eyeReference[i])<<i;
    }
    return eyeValue;
}