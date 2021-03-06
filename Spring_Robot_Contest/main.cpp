/**
 * @file manual_control.cpp
 * @brief CAN通信による制御信号を用いてアームを動作させるプログラム
 * @author Akihisa Watanabe
 * @date 2022.02.26
 */
#include <stdio.h>
#include "mbed.h"
#include "EC.h"
#include "CalPID.h"
#include "MotorController.h"

#define RESOLUTION 500      //モーターの分解能　データシート参照
#define DELTA_T 0.001       //pidの幅
#define DUTY_MAX 0.5        //duty上限
#define OMEGA_MAX 50        //ω上限
#define TIME_STOP 10.0


Ticker ticker;


void alert();
int move_arm(char option=0);

CAN can1(PB_8,PB_9);
Serial pc(USBTX,USBRX);


PwmOut arm_down(PA_12); 
PwmOut arm_up(PA_11);

PwmOut rack_pull(PC_8); 
PwmOut rack_push(PC_6);

PwmOut arm_rotate_1(PB_1); 
PwmOut arm_rotate_2(PB_15);

PwmOut PWM_TRIGER(PC_12); //超音波センサモジュールのTriger端子に入力する信号
InterruptIn GET_PWM(PC_10); //割り込み入力端子の設定．マイコンから出力したPWM信号をD9端子から取り込む． 

PwmOut servo(PC_7); //pin setting


//PID設定
CalPID rot_speed_pid(0.003,0,0.000013,DELTA_T,DUTY_MAX);
CalPID rot_duty_pid(0.003,0,0.000013,DELTA_T,DUTY_MAX);
CalPID rot_omega_pid(10.0,0,0.0100,DELTA_T,OMEGA_MAX);
Ec1multi rotEC(PC_10,PC_12,RESOLUTION); 

//モーター設定
MotorController motor_rot(PC_6,PC_8,DELTA_T, rotEC,rot_speed_pid,rot_omega_pid);

/**
 * @fn
 * @brief アームを上下させる関数
 * @param (option) 0:アームを上げる(初期位置), 1:アームを下げる
 * @return int 処理が成功した場合は1を，失敗した場合は0を返す．
 */
int move_arm(char option){
    arm_down = 0;
    arm_up = 0;
    double i=0;
    wait(5);
    
    switch (option){
        case 'a':
                arm_down = 0;
                arm_up = 0.5;
        case 'b':
                arm_down = 0.5;
                arm_up = 0;
        default:
            return 0;
    }
    wait(5);
    arm_down = 0;
    arm_up = 0;    
    return 1;
}

/**
 * @fn
 * @brief アームを回転させる関数
 * @param 
 * @return 
 */
int rotate_arm(int min_theta, int max_theta){
    arm_rotate_1 = 0;
    arm_rotate_2 = 0;
    int theta;
    wait(5);

    while (1){
        theta = rotEC.getDeg();
        if (theta < min_theta){
            arm_rotate_1 = 0.5;
            arm_rotate_2 = 0;            
        }
        if ((min_theta <= theta)&&(theta <= max_theta)){
            arm_rotate_1 = 0;
            arm_rotate_2 = 0;  
            break;
        }
        else {
            alert();
            return 0;
        }
    }
    wait(5);
    return 1;
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
            int i=0;
        case 'a':
            rack_pull = 0;
            rack_push = 0.5;
        case 'b':
            rack_pull = 0.5;
            rack_push = 0;
        default:
            return 0;
    }

    wait(5);
    return 1;
}

/**
 * @fn
 * @brief サーボモーターを動かす関数
 * @param (option) 0:蓋を開ける(初期位置), 1:蓋を閉める
 * @return int 処理が成功した場合は1を，失敗した場合は0を返す．
 */
int servo_motor(char option){
    servo.period_us(20000);  //周期設定
    while(1){
        servo.pulsewidth_us(500); //パルス幅変更 開いた状態
        wait(1);
        servo.pulsewidth_us(1500); //閉じた状態
        wait(3);
    }
    wait(5);
    return 1;
}

/**
 * @fn
 * @brief コントローラに異常を知らせる関数
 * @param 
 * @return 
 */
void alert(){
    //do nothing
    /*
    can_data[0]=0;//エラーコード
    can1.write(CANMessage(2,can_data,4));
    can_data[1]=1;
    */
}


/**
 * @fn
 * @brief CAN通信で制御信号を受信してアームを制御する関数
 */
void control(){
    CANMessage msg;
    int status;
    
    if (can1.read(msg)){
        if(msg.id == 0){
            if (msg.data[0]=='c'){
                if (msg.data[1] != 'a'){
                    status = move_arm(msg.data[1]);              
                }
                else if (msg.data[2] != 'a'){
                    int angle;
                    if (msg.data[2]=='b'){
                        angle = 40;
                    }
                    else if (msg.data[2]=='c'){
                        angle = 70;
                    }
                    else if (msg.data[2]=='d'){
                        angle = 100;
                    }
                    else {
                        alert();
                    }
                    status = rotate_arm(angle,angle+5);
                }
                else if (msg.data[3] != 'a'){
                    status = move_rack(msg.data[2]);
                }
                if (status == 0){
                    alert();
                }
            }
        }
    }
}

int main(){
    arm_down.period(0.05);
    arm_up.period(0.05);
    arm_rotate_1.period(0.05);
    arm_rotate_2.period(0.05);
    rack_pull.period(0.05);
    rack_push.period(0.05);
    move_arm(0);
    wait(10);
    wait(10);

    can1.frequency(100000);

    while(1){
        control();
        wait(0.5);  
    }
}