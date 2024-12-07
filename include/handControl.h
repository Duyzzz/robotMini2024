#ifndef HANDCONTROL_H
#define HANDCONTROL_H

void automation(unsigned int *encoderValue);
void handInLoop();
void handConfig();
void configMidload();
void startMidLoad();
void stopMidLoad();
void reverseMidload();
void unlockFiring();
void servoConfig();
void chooseUnlock();
void chooseColumn();
void modifyUnlockFireAngle(unsigned char angle);
void encoderReader();
void fireHold();
#endif