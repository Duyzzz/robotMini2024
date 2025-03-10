#ifndef MOVING_H
#define MOVING_H

#include "classes.h"
#include "angle.h"
#include <MiniPID.h>
// bool resetGo = true;
void motorConfig();
void robotDirectionConfig();
void goUp(uint8_t vLeft, uint8_t vRight);
void goDown(uint8_t vLeft, uint8_t vRight);
void goLeft(uint8_t vHead, uint8_t vBottom);
void goRight(uint8_t vHead, uint8_t vBottom);
void go(uint8_t expectedSpeed, char direction, bool *resetGo_, bool *requireSoftStop, uint16_t *currentSpeed);
void rotateLeft(uint8_t speed);
void rotateRight(uint8_t speed);
void brake(char direct, bool *tSet);
#endif