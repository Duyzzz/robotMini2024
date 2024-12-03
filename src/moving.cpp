#include "moving.h"

MiniPID robotDirection(0.1, 0.01, 0);

Motor topLeftMotor(1, 2, 4);
Motor topRightMotor(1, 2, 4);
Motor bottomLeftMotor(1, 2, 4);
Motor bottomRightMotor(1, 3, 5);

void goUp(uint8_t vLeft, uint8_t vRight)
{
    topLeftMotor.forward(vLeft);
    bottomLeftMotor.forward(vLeft);
    topRightMotor.forward(vRight);
    bottomRightMotor.forward(vRight);
}
void goDown(uint8_t vLeft, uint8_t vRight){
    topLeftMotor.reverse(vLeft);
    bottomLeftMotor.reverse(vLeft);
    topRightMotor.reverse(vRight);
    bottomRightMotor.reverse(vRight);
}
void goLeft(uint8_t vHead, uint8_t vBottom){
    topLeftMotor.reverse(vHead);
    bottomLeftMotor.forward(vBottom);
    topRightMotor.forward(vBottom);
    bottomRightMotor.reverse(vHead);
}
void goRight(uint8_t vHead, uint8_t vBottom){
    topLeftMotor.forward(vHead);
    bottomLeftMotor.reverse(vBottom);
    topRightMotor.reverse(vBottom);
    bottomRightMotor.forward(vHead);
}
void robotDirectionConfig(){
    robotDirection.setOutputLimits(-50, 50);
    robotDirection.setOutputRampRate(10);
    robotDirection.setOutputFilter(0.1);
}
void motorConfig(){
    topLeftMotor.config();
    topRightMotor.config();
    bottomLeftMotor.config();
    bottomRightMotor.config();
}
void rotateLeft(uint8_t speed){
    topLeftMotor.reverse(speed);
    bottomLeftMotor.reverse(speed);
    topRightMotor.forward(speed);
    bottomRightMotor.forward(speed);
}
void rotateRight(uint8_t speed){
    topLeftMotor.forward(speed);
    bottomLeftMotor.forward(speed);
    topRightMotor.reverse(speed);
    bottomRightMotor.reverse(speed);
}
void go(uint8_t expectedSpeed, char direction){
    static int directionAim = readAngle();
    int out = robotDirection.getOutput(readAngle(), directionAim);
    int v1 = constrain(expectedSpeed + out, 0, 255);
    int v2 = constrain(expectedSpeed - out, 0, 255);
    if(direction == 'u'){
        goUp(v1, v2);
    }else if(direction == 'd'){
        goDown(v1, v2);
    }else if(direction == 'l'){
        goLeft(v1, v2);
    }else if(direction == 'r'){
        goRight(v1, v2);
    }
}