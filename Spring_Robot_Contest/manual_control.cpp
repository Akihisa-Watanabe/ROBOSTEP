/**
 * @file manual_control.cpp
 * @brief CAN通信による制御信号を用いてアームを動作させるプログラム
 * @author Akihisa Watanabe
 * @date 2022.02.26
 */
#include <stdio.h>
#include "mbed.h"
#include "EC.h"

CAN can1(p30,p29);
Ticker ticker;

PwmOut arm_down(PA_12); //0.5出力 で動く、０出力で動かない
PwmOut arm_up(PA_11);//0.5出力 で動く、０出力で動かない
arm_down.period(50);
arm_up.period(50);

PwmOut rack_pull(PC_8); 
PwmOut rack_push(PC_6);
rack_pull.period(50);
rack_push.period(50);

PwmOut arm_rotate_1(PC_8); 
PwmOut arm_rotate_2(PC_6);
arm_rotate_1.period(50);
arm_rotate_2.period(50);


char can_data[4]={1,0,0,0};//CAN送信用の配列4byte

/**
 * @fn
 * @brief アームを上下させる関数
 * @param (option) 0:アームを上げる(初期位置), 1:アームを下げる
 * @return int 処理が成功した場合は1を，失敗した場合は0を返す．
 */
int move_arm(char option){
    arm_down = 0;
    arm_up = 0;
    wait(5);
    switch (option){
        case 0:
            arm_down = 0;
            arm_up = 0.5;
        case 1:
            arm_down = 0.5;
            arm_up = 0;
        default:
            return 0;
    }

    wait(300);
    return 1;
}

/**
 * @fn
 * @brief アームを回転させる関数
 * @param 
 * @return 
 */
int rotate_arm(char option){

}

/**
 * @fn
 * @brief ラックを押し引きする関数
 * @param (option) 0:ラックを引く(初期位置), 1:ラックを押し出す
 * @return int 処理が成功した場合は1を，失敗した場合は0を返す．
 */
int move_rack(char option){
    rack_pull = 0;
    rack_push = 0;
    wait(5);
    switch (option){
        case 0:
            rack_pull = 0;
            rack_push = 0.5;
        case 1:
            rack_pull = 0.5;
            rack_push = 0;
        default:
            return 0;
    }

    wait(300);
    return 1;
}

/**
 * @fn
 * @brief コントローラに異常を知らせる関数
 * @param 
 * @return 
 */
void alert(){
    can_data[0]=0;//エラーコード
    if(can1.write(CANMessage(2,can_data,4))){
        led = 1
    }
    led = 0;
    can_data[1]=1;
}


/**
 * @fn
 * @brief CAN通信で制御信号を受信してアームを制御する関数
 */
void control(){
    CANMessage msg;
    int status;
    
    if (can1.read(msg)){
        if(msg.id == 2){
            if (msg.data[0] != 0){
                status = move_arm(msg.data[0]);              
            }
            else if (msg.data[1] != 0){
                status = rotate_arm(msg.data[1])
            }
            else if (msg.data[2] != 0){
                status = move_rack(msg.data[2]);
            }

            if (status == 0){
                alert();
            }
        }
    }
}

int main(){
    wait(10);
    ticker.attach(&control,0.001);
}