#include "classes.h"



void DigitalControl :: config(int type){
    pinMode(pin, type);
}
void DigitalControl :: action(){
    digitalWrite(pin,act); 
}
void DigitalControl :: stop(){
    digitalWrite(pin, !act); 
}




void Motor :: forward(int speed){
    analogWrite(Pwm, speed); 
    digitalWrite(In1,1); 
    digitalWrite(In2,0);
}
void Motor :: reverse(int speed){
    analogWrite(Pwm, speed); 
    digitalWrite(In1,0); 
    digitalWrite(In2,1);
}
void Motor :: stop(){
    digitalWrite(In1,0); 
    digitalWrite(In2,0);
}
void Motor :: brake(){
    digitalWrite(In1,1); 
    digitalWrite(In2,1);
}
void Motor :: config(){
    pinMode(In1, OUTPUT);
    pinMode(In2, OUTPUT);
    pinMode(Pwm, OUTPUT);
}