#include "digital.h"

DigitalControl ballGetter(2, 0);
DigitalControl higherLoad(36, 0);

void configLoad(){
    ballGetter.config(OUTPUT);
    higherLoad.config(OUTPUT);
    ballGetter.stop();
    higherLoad.stop();
}
void toggleTakeBallState(){
    Serial.println(ballGetter.currentState());
    if(ballGetter.currentState()){
        stopTakeBall();
    }else {
        takeBall();
    }
}
bool getUpperLoadState(){
    return higherLoad.currentState();
}
void takeBall(){
    ballGetter.action();
}
void stopTakeBall(){
    ballGetter.stop();
}
void loadBallToFire(){
    higherLoad.action();
}
void stopLoadBallToFire(){
    higherLoad.stop();
}