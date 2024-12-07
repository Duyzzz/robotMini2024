#include "moving.h"

static int directionAim = 0;
MiniPID robotDirection(1.65, 0.2, 0.1);
// MiniPID robotDirection(1.6, 0.03, 0.01);

Motor bottomRightMotor(41, 49, 6); // ok
Motor topRightMotor(45, 53, 4);
Motor topLeftMotor(37, 29, 8); // ok
Motor bottomLeftMotor(39, 31, 7); // ok 




void goUp(uint8_t vLeft, uint8_t vRight)
{
    topLeftMotor.forward(vLeft);
    bottomLeftMotor.forward(vLeft);
    topRightMotor.forward(vRight);
    bottomRightMotor.forward(constrain(vRight + 20, 0, 255));
    Serial.println("go up");
}
void goDown(uint8_t vLeft, uint8_t vRight){
    topLeftMotor.reverse(vLeft);
    bottomLeftMotor.reverse(vLeft);
    topRightMotor.reverse(vRight);
    bottomRightMotor.reverse(constrain(vRight + 20, 0, 255));
    Serial.println("go down");
}
void goLeft(uint8_t vHead, uint8_t vBottom){
    // Serial.print("after: ");
    // Serial.print(vHead);
    // Serial.print(" ");
    // Serial.println(vBottom);
    topLeftMotor.reverse(vHead);
    bottomLeftMotor.forward(vBottom);
    topRightMotor.forward(vBottom);
    bottomRightMotor.reverse(constrain(vHead + 20, 0, 255));
    Serial.println("go left");
}
void goRight(uint8_t vHead, uint8_t vBottom){
    topLeftMotor.forward(vHead);
    bottomLeftMotor.reverse(vBottom);
    topRightMotor.reverse(vBottom);
    bottomRightMotor.forward(constrain(vHead + 20, 0, 255));
    Serial.println("go right");
}
void robotDirectionConfig(){
    robotDirection.setOutputLimits(-100, 100);
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
}
void rotateLeft(uint8_t speed){
    topLeftMotor.reverse(speed);
    bottomLeftMotor.reverse(speed);
    topRightMotor.forward(speed);
    bottomRightMotor.forward(constrain(speed + 20, 0, 255));
}
void rotateRight(uint8_t speed){
    topLeftMotor.forward(speed);
    bottomLeftMotor.forward(speed);
    topRightMotor.reverse(speed);
    bottomRightMotor.reverse(constrain(speed + 20, 0, 255));
}
void go(uint8_t expectedSpeed, char direction, bool *resetGo_, bool *stopRequire, uint16_t *currentSpeed){
    
    static long timeIncrease = millis();
    static uint16_t curSp = 0;
    if(*resetGo_){
        directionAim = readAngle();
        robotDirection.setSetpoint(directionAim);
        *resetGo_ = false;
        robotDirection.reset();
        Serial.println("get new setpoint");
        timeIncrease = millis();
        curSp = *currentSpeed;
    }
    // float kp = 2.2 - (float)(*currentSpeed)/255*2.2 - 1.65;   // (1.65 - 3.2)
    // robotDirection.setP(kp);
    // Serial.print(kp);
    int speed = 0;
    if(*stopRequire){
        speed = curSp - map(millis() - timeIncrease, 0, 400, 0, curSp);
        speed = constrain(speed, 0, expectedSpeed);
        if(speed <= 0){
            *stopRequire = false;
        }
    }else {
        speed = map(millis() - timeIncrease, 0, 50, 0, expectedSpeed);
        speed = constrain(speed, 0, expectedSpeed);

    }
    *currentSpeed = speed;
    // Serial.print(speed);
    // Serial.print(" ");
    // Serial.print(curSp);
    // Serial.print(" ");
    // Serial.print(millis() - timeIncrease);
    // Serial.print(" ");
    int v1 = 0, v2 = 0;
    int out = robotDirection.getOutput(readAngle(), directionAim);
    if(direction == 'l' || direction == 'r'){
        v1 = constrain(speed + out, 0, 255);
        v2 = constrain(speed - out, 0, 255);
    }else {
        v1 = constrain(speed + out, 0, 255);
        v2 = constrain(speed - out, 0, 255);
    }
    
    // Serial.print(v1);
    // Serial.print(" ");
    // Serial.print(v2);
    // Serial.print(directionAim);
    // Serial.print(" ");
    // Serial.print(readAngle());
    // Serial.print(" ");
    // Serial.println(out);
    // Serial.println();

    /* the code below fine with go down
    
    
    
    
    int vOffset = 30 - map(millis() - timeIncrease, 0, 1000, 0, 30);
    vOffset = constrain(vOffset, 0, 30);
    Serial.println(vOffset);
    if(direction == 'u'){
        goUp(v1, v2);
        // goUp(250, 250);
        // Serial.println("upup");
    }else if(direction == 'd'){
        goDown(v2 - vOffset, v1 + 0.2*vOffset);
    }else if(direction == 'l'){
        goLeft(v2, v1 + vOffset);
    }else if(direction == 'r'){
        goRight(v1, v2 + vOffset);
    }
    
    */
    int vOffset = 30 - map(millis() - timeIncrease, 0, 500, 0, 30);
    vOffset = constrain(vOffset, 0, 30);
    if(direction == 'u'){
        goUp(v1, v2);
        // goUp(250, 250);
        // Serial.println("upup");
    }else if(direction == 'd'){
        goDown(constrain(v2 - vOffset, 0, 255), v1 + 0.2*vOffset);
    }else if(direction == 'l'){
        goLeft(v2, v1);
    }else if(direction == 'r'){
        vOffset = 50 - map(millis() - timeIncrease, 0, 500, 0, 50);
        vOffset = constrain(vOffset, 0, 50);
        goRight(constrain(v1 - vOffset, 0, 255), constrain(v2 +0.5*vOffset, 0, 255));
    }
}



void brake(char direct, bool *tSet){
    static long timeRest = 0;
    if(*tSet){
        *tSet = false;
        timeRest = millis();
        Serial.println("brake");
    }
    if(millis() - timeRest < 200){
        if(direct == 'u'){
            goDown(130, 130);
        }else if(direct == 'd'){
            goUp(130, 130);
        }else if(direct == 'l'){
            goRight(130, 130);
        }else if(direct == 'r'){
            goLeft(130, 130);
        }
    }else {
        goUp(0, 0);
    }
}