#include "handControl.h"
#include "moving.h"
#include "PS3_Index.h"
bool resetGo = true;
void ps3() {
    static unsigned long defendTime = millis();
    if(millis() - defendTime > 10){
        static uint32_t  combined_data = 0; 
        Serial1.print('p'); 
        // uint8_t byte_[4] = {0x00,0x00,0x00,0x00};
        delay(2); 
        if(Serial1.available()){
            char haha = Serial1.read(); 
            //   Serial.println(haha);
            if(haha == '}'){                // Kiem tra khoa 
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

void handInLoop(){
    ps3();
    delay(2);
    if(button_l2){
        rotateLeft(200);
    }else if(button_r2){
        rotateRight(200);
    }else if(button_up){
        // goUp(255, 255);
        go(250, 'u', &resetGo);
    }else if(button_down){
        // goDown(255, 255);
        go(250, 'd', &resetGo);
    }else if(button_left){
        // goLeft(255, 255);
        go(250, 'l', &resetGo);
    }else if(button_right){
        // goRight(255, 255);
        go(250, 'r', &resetGo);
    }else{
        goUp(0, 0);
        resetGo = true;
    }
    if(button_x){
        Serial3.write('r');
    }
}