#include "moving.h"

static int directionAim = 0;
MiniPID robotDirection(1.6, 0.03, 0.01);

Motor bottomRightMotor(41, 49, 6); // ok
Motor topRightMotor(45, 53, 4);
Motor topLeftMotor(37, 29, 8); // ok
Motor bottomLeftMotor(39, 31, 7); // ok 

Motor upperRulo(33, 25, 10);
Motor LowerRulo(35, 27, 9);



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
    robotDirection.setOutputLimits(-30, 30);
    robotDirection.setOutputRampRate(10);
    robotDirection.setOutputFilter(0.1);
    robotDirection.setSetpointRange(90);
    robotDirection.setMaxIOutput(10);
    robotDirection.setF(0);
}
void motorConfig(){
    topLeftMotor.config();
    topRightMotor.config();
    bottomLeftMotor.config();
    bottomRightMotor.config();
    upperRulo.config();
    LowerRulo.config();
}
void testRulo(){
    upperRulo.forward(250);
    LowerRulo.forward(250);
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
void go(uint8_t expectedSpeed, char direction, bool *resetGo_){
    if(*resetGo_){
        directionAim = readAngle();
        robotDirection.setSetpoint(directionAim);
        *resetGo_ = false;
        robotDirection.reset();
        Serial.println("get new setpoint");
    }
    int v1 = 0, v2 = 0;
    int out = robotDirection.getOutput(readAngle(), directionAim);
    if(direction == 'l' || direction == 'r'){
        v1 = constrain(expectedSpeed + out, 220, 255);
        v2 = constrain(expectedSpeed - out, 220, 255);
    }else {
        v1 = constrain(expectedSpeed + out, 0, 255);
        v2 = constrain(expectedSpeed - out, 0, 255);
    }
    
    Serial.print(v1);
    Serial.print(" ");
    Serial.println(v2);
    // Serial.print(directionAim);
    // Serial.print(" ");
    // Serial.print(readAngle());
    // Serial.print(" ");
    // Serial.println(out);

    if(direction == 'u'){
        goUp(v1, v2);
    }else if(direction == 'd'){
        goDown(v2, v1);
    }else if(direction == 'l'){
        goLeft(v2, v1);
    }else if(direction == 'r'){
        goRight(v1, v2);
    }
}