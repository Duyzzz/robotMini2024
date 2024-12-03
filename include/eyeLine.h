#ifndef __EYELINE_H_
#define __EYELINE_H_

#include "Arduino.h"

uint8_t eyeList[8] = {54U, 55U, 56U, 57U, 58U, 59U, 60U, 61U};
uint8_t eyeReference[8] = {1, 2, 3, 4, 5, 6, 7, 8};
bool checkEye(uint8_t eye, uint16_t reference);
int readEyes();

#endif