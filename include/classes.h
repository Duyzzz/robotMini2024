#ifndef CLASSES_H
#define CLASSES_H

#include "Arduino.h"
#include <inttypes.h>

class DigitalControl{
    private:
        uint8_t pin; 
        bool act; 
        bool isActive = false;
    public: 
        DigitalControl(int pinn, bool actt):pin(pinn),act(actt){}
        void config(int type);
        void action();
        void stop();
        bool currentState();
}; 

class Motor{
    protected: 
        int In1; 
        int In2; 
        int Pwm; 
    public:     
    Motor(int in1, int in2, int pwm):In1(in1),In2(in2),Pwm(pwm){}
    void config();
    void forward(int speed);
    void reverse(int speed);
    void stop();
    void brake();
};
#endif