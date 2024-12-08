#include "handControl.h"
#include "moving.h"
#include "PS3_Index.h"
#include "digital.h"
#include <Servo.h>
unsigned long timeResetWaitingBall = 0;
unsigned int encoder = 0;
#define UNLOCK 0
#define COLUMN 1
bool enableToggleState = true;
bool startWaiting = false;
bool choosingFire = UNLOCK;
const char autoPin = 47;
char robotDirectionSet = 's'; // stop;
Servo fireChoosingServo;
Servo unlockFireServo;
uint16_t robotCurrentSpeed = 0;
bool resetGo = true;
bool softStopResetGo = true;
bool requireSoftStop = false;
bool stopOnce = true;
bool enableBrake = false;
bool earlyStop = false;
uint8_t fireGunSpeed = 0;
char command = 'n'; // w waiting for auto, n for hand control, a for automation.
int threading = 0;
bool config = true;
bool waitingBall = false;
const char ballSensor = 43;
unsigned long timeDelayForStop = 0;
char delayStop = 'a';
Motor midLoad(35, 27, 9);
void ps3()
{
    static unsigned long defendTime = millis();
    if (millis() - defendTime > 10)
    {
        static uint32_t combined_data = 0;
        Serial1.print('p');
        // uint8_t byte_[4] = {0x00,0x00,0x00,0x00};
        delay(2);
        if (Serial1.available())
        {
            char haha = Serial1.read();
            //   Serial.println(haha);
            if (haha == '}')
            { // Kiem tra khoa
                uint8_t byte_1 = Serial1.read();
                uint8_t byte_2 = Serial1.read();
                uint8_t byte_3 = Serial1.read();
                uint8_t byte_4 = Serial1.read();
                combined_data = ((uint32_t)byte_1 << 24) | ((uint32_t)byte_2 << 16) | ((uint32_t)byte_3 << 8) | byte_4;
            }
        }
        // Serial.println(bt_in);
        bt_in = combined_data;
        defendTime = millis();
    }
}
void choosingShaking()
{
    static long timeShaking = millis();
    static bool state = true;
    if (choosingFire == UNLOCK)
    {
        if (millis() - timeShaking > 200)
        {
            state = !state;
            timeShaking = millis();
        }
        if (state)
        {
            fireChoosingServo.write(30);
        }
        else
        {
            fireChoosingServo.write(50);
        }
    }
    else
    {
        if (millis() - timeShaking > 200)
        {
            state = !state;
            timeShaking = millis();
        }
        if (state)
        {
            fireChoosingServo.write(130);
        }
        else
        {
            fireChoosingServo.write(150);
        }
    }
}
void automation(unsigned int *encoderVal)
{
    Serial.println("setup automation");
    unsigned long timeAuto = millis();
    toggleTakeBallState();
    while (true)
    {
        // Serial.println("running in automationMode");
        // Serial.println(*encoderVal);
        if (threading == 0)
        {
            ps3();
            if (millis() - timeAuto < 7000 && *encoderVal < 6000)
            {
                Serial.println("go down");
                go(220, 'd', &resetGo, &requireSoftStop, &robotCurrentSpeed);
                robotDirectionSet = 'd';
                if (*encoderVal > 5500)
                {
                    goUp(20, 20);
                }
            }
            else if (millis() - timeAuto < 13000)
            {
                Serial.println("go down");
                go(160, 'd', &resetGo, &requireSoftStop, &robotCurrentSpeed);
                if (*encoderVal > 8800)
                {
                    earlyStop = true;
                }
                robotDirectionSet = 'd';
            }
            else
            {
                goUp(0, 0);
                brake('d', &enableBrake);
                Serial.println("end time");
                robotDirectionSet = 's';
            }
            if (earlyStop | button_up | button_down | button_left | button_right | button_select | button_triangle | button_square | button_circle | button_x | button_r1 | button_l1 | button_r2 | button_l2)
            {
                goUp(0, 0);
                brake('d', &enableBrake);
                Serial.println("out auto");
                robotDirectionSet = 's';
                threading = 1;
                modifyUnlockFireAngle(90);
            }
        }
        else if (threading == 1)
        {
            static int set = 0;
            static bool exitCode = false;
            if (config)
            {
                set = readAngle() + 90;
                config = false;
            }
            int out = 255 - map(readAngle(), set - 90, set, 80, 100);
            rotateRight(constrain(out, 80, 250));
            if (abs(readAngle() - set) < 5)
            {
                config = true;
                threading = 2;
            }
        }
        else if (threading == 2)
        {
            static bool resetGoNow = true;
            static bool softStop = false;
            if (config)
            {
                encoder = 0;
                config = false;
            }
            if (encoder < 2700)
            {
                go(180, 'u', &resetGoNow, &softStop, &robotCurrentSpeed);
            }
            else if (encoder < 2800)
            {
                goDown(70, 70);
                resetGoNow = true;
            }
            else if (encoder < 3000)
            {
                go(160, 'u', &resetGoNow, &softStop, &robotCurrentSpeed);
            }
            else
            {
                goUp(0, 0);
                choosingShaking();
                ps3();
                if (button_l2)
                {
                    requireSoftStop = false;
                    rotateLeft(200);
                    stopOnce = true;
                    robotDirectionSet = 'q'; // rotate left
                }
                else if (button_r2)
                {
                    rotateRight(200);
                    requireSoftStop = false;
                    stopOnce = true;
                    robotDirectionSet = 'w'; // rotate right
                }
                else if (button_up)
                {
                    // goUp(255, 255);
                    // Serial.println("up");
                    go(250, 'u', &resetGo, &requireSoftStop, &robotCurrentSpeed);
                    requireSoftStop = false;
                    stopOnce = true;
                    robotDirectionSet = 'u';
                }
                else if (button_down)
                {
                    requireSoftStop = false;
                    // goDown(255, 255);
                    stopOnce = true;
                    robotDirectionSet = 'd';
                    go(250, 'd', &resetGo, &requireSoftStop, &robotCurrentSpeed);
                }
                else if (button_left)
                {
                    requireSoftStop = false;
                    // goLeft(255, 255);
                    stopOnce = true;
                    robotDirectionSet = 'l';
                    go(250, 'l', &resetGo, &requireSoftStop, &robotCurrentSpeed);
                }
                else if (button_right)
                {
                    // goRight(255, 255);
                    requireSoftStop = false;
                    stopOnce = true;
                    robotDirectionSet = 'r';
                    go(250, 'r', &resetGo, &requireSoftStop, &robotCurrentSpeed);
                }
                else
                {
                    if (stopOnce)
                    {
                        requireSoftStop = true;
                        softStopResetGo = true;
                        resetGo = true;
                        stopOnce = false;
                        enableBrake = true;
                        Serial.println("check stop once");
                    }
                    if (requireSoftStop)
                    {
                        if (robotDirectionSet == 'u')
                        {
                            // go(250, 'u', &softStopResetGo, &requireSoftStop, &robotCurrentSpeed);
                            brake('u', &enableBrake);
                        }
                        else if (robotDirectionSet == 'd')
                        {
                            brake('d', &enableBrake);
                            // go(250, 'd', &softStopResetGo, &requireSoftStop, &robotCurrentSpeed);
                        }
                        else if (robotDirectionSet == 'l')
                        {
                            brake('l', &enableBrake);
                            // go(250, 'l', &softStopResetGo, &requireSoftStop, &robotCurrentSpeed);
                        }
                        else if (robotDirectionSet == 'r')
                        {
                            brake('r', &enableBrake);
                            // go(250, 'r', &softStopResetGo, &requireSoftStop, &robotCurrentSpeed);
                        }
                        else if (robotDirectionSet == 'q')
                        {
                            goUp(0, 0);
                        }
                        else if (robotDirectionSet == 'w')
                        {
                            goUp(0, 0);
                        }
                    }
                }
                if (button_triangle)
                {
                    loadBallToFire();
                    startMidLoad();
                    loadBallToFire();
                }
                if (button_circle)
                {
                    threading = 3;
                    break;
                }
            }
        }
    }
    command = 'n';
}

void servoConfig()
{
    fireChoosingServo.attach(13);
    unlockFireServo.attach(44);
    fireChoosingServo.write(50);
    // unlockFireServo.write(75);
}
void configMidload()
{
    midLoad.config();
}
void encoderReader()
{
    encoder++;
}
void handConfig()
{
    robotDirectionConfig();
    angleConfig();
    configLoad();
    motorConfig();
    configMidload();
    servoConfig();
    chooseUnlock();
    modifyUnlockFireAngle(160);
    pinMode(autoPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(20), encoderReader, FALLING);
    Serial2.begin(115200);
    pinMode(ballSensor, INPUT_PULLUP);

    // chooseColumn();
}

void modifyUnlockFireAngle(unsigned char angle)
{
    unlockFireServo.write(angle);
}
void chooseUnlock()
{
    fireChoosingServo.write(40);
}
void chooseColumn()
{
    fireChoosingServo.write(140);
}
void startMidLoad()
{
    midLoad.forward(255);
}
void unlockFiring()
{
    startMidLoad();
    stopTakeBall();
    loadBallToFire();
    // TODO control servo to unlock
}
void stopMidLoad()
{
    midLoad.forward(0);
}
void reverseMidload()
{
    midLoad.reverse(255);
}

void handInLoop()
{
    // Serial.println(command);
    if (command == 'w')
    {
        if (!digitalRead(autoPin))
        {
            command = 'a';
            delay(20);
            while (!digitalRead(autoPin))
                ;
            enableBrake = true;
        }
    }
    else if (command == 'a')
    {
        ps3();
        automation(&encoder);
        modifyUnlockFireAngle(90);
    }
    else if (command == 'n')
    {
        ps3();
        if (button_start)
        {
            command = 'w';
            goUp(0, 0);
            stopOnce = true;
            robotDirectionSet = 's';
            stopTakeBall();
            stopLoadBallToFire();
        }
        if (button_l2)
        {
            requireSoftStop = false;
            rotateLeft(200);
            stopOnce = true;
            robotDirectionSet = 'q'; // rotate left
        }
        else if (button_r2)
        {
            rotateRight(200);
            requireSoftStop = false;
            stopOnce = true;
            robotDirectionSet = 'w'; // rotate right
        }
        else if (button_up)
        {
            // goUp(255, 255);
            // Serial.println("up");
            go(250, 'u', &resetGo, &requireSoftStop, &robotCurrentSpeed);
            requireSoftStop = false;
            stopOnce = true;
            robotDirectionSet = 'u';
        }
        else if (button_down)
        {
            requireSoftStop = false;
            // goDown(255, 255);
            stopOnce = true;
            robotDirectionSet = 'd';
            go(250, 'd', &resetGo, &requireSoftStop, &robotCurrentSpeed);
        }
        else if (button_left)
        {
            requireSoftStop = false;
            // goLeft(255, 255);
            stopOnce = true;
            robotDirectionSet = 'l';
            go(250, 'l', &resetGo, &requireSoftStop, &robotCurrentSpeed);
        }
        else if (button_right)
        {
            // goRight(255, 255);
            requireSoftStop = false;
            stopOnce = true;
            robotDirectionSet = 'r';
            go(250, 'r', &resetGo, &requireSoftStop, &robotCurrentSpeed);
        }
        else
        {
            if (stopOnce)
            {
                requireSoftStop = true;
                softStopResetGo = true;
                resetGo = true;
                stopOnce = false;
                enableBrake = true;
                Serial.println("check stop once");
            }
            if (requireSoftStop)
            {
                if (robotDirectionSet == 'u')
                {
                    // go(250, 'u', &softStopResetGo, &requireSoftStop, &robotCurrentSpeed);
                    brake('u', &enableBrake);
                }
                else if (robotDirectionSet == 'd')
                {
                    brake('d', &enableBrake);
                    // go(250, 'd', &softStopResetGo, &requireSoftStop, &robotCurrentSpeed);
                }
                else if (robotDirectionSet == 'l')
                {
                    brake('l', &enableBrake);
                    // go(250, 'l', &softStopResetGo, &requireSoftStop, &robotCurrentSpeed);
                }
                else if (robotDirectionSet == 'r')
                {
                    brake('r', &enableBrake);
                    // go(250, 'r', &softStopResetGo, &requireSoftStop, &robotCurrentSpeed);
                }
                else if (robotDirectionSet == 'q')
                {
                    goUp(0, 0);
                }
                else if (robotDirectionSet == 'w')
                {
                    goUp(0, 0);
                }
            }
        }
        if (button_x)
        {
            if (enableToggleState)
            {
                startMidLoad();
                toggleTakeBallState();
                enableToggleState = false;
            }
            // Serial3.write('r');
        }
        else if (button_triangle)
        {
            if (waitingBall == true)
            {
                // TODO
                Serial.println("waiting to fire");
                loadBallToFire();
                startWaiting = true;
                timeResetWaitingBall = millis();
            }
            else
            {
                Serial.println("no");
                delayStop = 'a';
            }
        }
        else if (button_l1)
        {
            if (enableToggleState)
            {
                if (fireGunSpeed == 69)
                {
                    // byte bytes[3] = {0x01, 0x06, 0x04};
                    Serial2.write(0x01);
                    fireGunSpeed = 0;
                    Serial.println("set speed 0");
                }
                else if (fireGunSpeed == 77 || fireGunSpeed == 0)
                {
                    // byte bytes[3] = {0x69, 0x06, 0x04};
                    Serial2.write(0x69);
                    Serial.println("set speed 69");
                    fireGunSpeed = 69;
                }
                enableToggleState = false;
            }
        }
        else if (button_r1)
        {
            if (enableToggleState)
            {
                if (fireGunSpeed == 77)
                {
                    // byte bytes[3] = {0x01, 0x06, 0x04};
                    Serial2.write(0x01);
                    fireGunSpeed = 0;
                    Serial.println("speed 0");
                }
                else if (fireGunSpeed == 69 || fireGunSpeed == 0)
                {
                    // byte bytes[3] = {0x77, 0x06, 0x04};
                    Serial2.write(0x77);
                    Serial.println("speed 77");
                    fireGunSpeed = 77;
                }
                enableToggleState = false;
            }
        }
        else if (button_square)
        {
            if (enableToggleState)
            {
                if (choosingFire == UNLOCK)
                {
                    chooseColumn();
                    choosingFire = COLUMN;
                }
                else
                {
                    chooseUnlock();
                    choosingFire = UNLOCK;
                }
                enableToggleState = false;
            }
        }
        else
        {
            enableToggleState = true;
        }
        if (getUpperLoadState())
        {
            choosingShaking();
        }
        fireHold();
    }
}
void fireHold()
{
    if (digitalRead(ballSensor) == LOW && waitingBall == false)
    {
        if (delayStop == 'a')
        {
            Serial.println("in a");
            timeDelayForStop = millis();
            loadBallToFire();
            delayStop = 'b';
        }
    }
    else if (delayStop == 'b')
    {
        Serial.println("in b");
        if (millis() - timeDelayForStop > 25)
        {
            Serial.println("in c");
            stopMidLoad();
            stopLoadBallToFire();
            waitingBall = true;
            delayStop = 'c';
            timeDelayForStop = millis();
        }
    }
    else if (delayStop == 'c')
    {
        if (millis() - timeDelayForStop > 300)
        {
            delayStop = 'd';
            Serial.println("in d");
        }
    }
    else if (millis() - timeResetWaitingBall > 1200 && startWaiting == true)
    {
        stopLoadBallToFire();
        startMidLoad();
        Serial.println("stop delay, start midload");
        startWaiting = false;
    }
}