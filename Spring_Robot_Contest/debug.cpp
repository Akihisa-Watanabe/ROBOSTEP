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

unsigned int Count;
unsigned char Flag;
double dist;

Ticker ticker;
Timer ActiveTime; //タイマー計測用変数

void RiseEcho();
int move_arm(char option=0);
void FallEcho();
void auto_arm(double threshold=100);
int move_arm(char option);
int rotate_arm(int min_theta=0, int max_theta=0);
int move_rack(char option=0);
int servo_motor();
void control();

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

void RiseEcho(){
    ActiveTime.start();
}
 
void FallEcho(){
    unsigned long ActiveWidth;
    ActiveTime.stop();
    ActiveWidth = ActiveTime.read_us();
    dist = ActiveWidth * 0.0170; //音速：0.034cm/us　
    ActiveTime.reset();
    Flag = 1; //フラグのリセット
}

void auto_arm(double threshold){
    GET_PWM.rise(&RiseEcho);
    GET_PWM.fall(&FallEcho);
    
    PWM_TRIGER.period(0.1);
    PWM_TRIGER.write(0.01f);
    
    Count=0;
    Flag= 0;
    ActiveTime.reset();
    if (Flag==1){
        if (dist < threshold){ //測定キョリが基準値よりも小さくなった場合
            move_arm(1);
        }
        else{
            move_arm(0);
        }
    }
}
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
        case 0:
            while(i<=0.5){
                i+=0.1;
                arm_down = 0;
                arm_up = i;
            }
        case 1:
            while(i<=0.5){
                i+=0.1;
                arm_down = i;
                arm_up = 0;
            }
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
    int i=0;
    int theta;
    wait(5);

    while (1){
        theta = rotEC.getDeg();
        if (i<=0.5){
            i+=0.1;
        }
        if (theta < min_theta){
            arm_rotate_1 = i;
            arm_rotate_2 = 0;            
        }
        if ((min_theta <= theta)&&(theta <= max_theta)){
            arm_rotate_1 = 0;
            arm_rotate_2 = 0;  
            break;
        }
        else {
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
        case 0:
            while(i<=0.5){
                i+=0.1;
                rack_pull = 0;
                rack_push = i;
            }
        case 1:
            while(i<=0.5){
                i+=0.1;
                rack_pull = i;
                rack_push = 0;
            }
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
int servo_motor(){
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

void print_result(int status){
    if (status==0){
        printf("Success.\n");
    }
    else{
        printf("Error.\n");
    }    
}

/**
 * @fn
 * @brief CAN通信で制御信号を受信してアームを制御する関数
 */
void control(){
    CANMessage msg;
    int status,angle=0;

    printf("Arm up-down test.\n");
    print_result(move_arm(0));  
    print_result(move_arm(1));  

    printf("Arm rotate test.\n");
    angle=0;
    print_result(rotate_arm(angle,angle+5));
    angle=40;
    print_result(rotate_arm(angle,angle+5));

    printf("Rack move test.\n");
    print_result(move_rack(0));
    print_result(move_rack(1));

    //printf("Ultrasonic Sensor test.\n")
    //auto_arm(100);
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
    ticker.attach(&control,0.001);
}