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

CAN can1(PB_8,PB_9);
char can_data[4]={1,0,0,0};//CAN送信用の配列4byte
Ticker ticker;

PwmOut arm_down(PA_12); //0.5出力 で動く、０出力で動かない
PwmOut arm_up(PA_11);//0.5出力 で動く、０出力で動かない
arm_down.period(50);
arm_up.period(50);

PwmOut rack_pull(PC_8); 
PwmOut rack_push(PC_6);
rack_pull.period(50);
rack_push.period(50);

PwmOut arm_rotate_1(PB_1); 
PwmOut arm_rotate_2(PB_15);
arm_rotate_1.period(50);
arm_rotate_2.period(50);

PwmOut PWM_TRIGER(PC_12); //超音波センサモジュールのTriger端子に入力する信号
InterruptIn GET_PWM(PC_10); //割り込み入力端子の設定．マイコンから出力したPWM信号をD9端子から取り込む． 

PwmOut servo(PC_7); //pin setting


Timer ActiveTime; //タイマー計測用変数


unsigned int Count;
unsigned char Flag;
double dist;

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
 * @brief encoder program
 * @param 
 * @return 
 */
//maxon DC motor 271566 角度制御を行う


#define RESOLUTION 500      //モーターの分解能　データシート参照
#define DELTA_T 0.001       //pidの幅
#define DUTY_MAX 0.5        //duty上限
#define OMEGA_MAX 50        //ω上限
#define TIME_STOP 10.0

Serial pc(USBTX,USBRX);

//PID設定
CalPID rot_speed_pid(0.003,0,0.000013,DELTA_T,DUTY_MAX);
//速度制御のPID,係数はモーターごとに調整の必要あり
CalPID rot_duty_pid(0.003,0,0.000013,DELTA_T,DUTY_MAX);
//角度制御のPID,モーターに出力されたduty比からフィードバックの場合
CalPID rot_omega_pid(10.0,0,0.0100,DELTA_T,OMEGA_MAX);
//角度制御のPID,SpeedControllerのωからフィードバックの場合

//今回は関係ないけどただのメモ
//角度制御のPIDに複数種類あり
//1.ωの最大値を指定して制御するパターン
//2.角度だけ設定して急いで動かすパターン

//エンコーダ設定
/*          (A層,B層,分解能)
Ec1multi EC(p7,p8,RESOLUTION);  //1逓倍用classエンコーダ1回転で1024
            or
Ec2multi EC(p7,p8,RESOLUTION);  //2逓倍用classエンコーダ1回転で2048
            or      */
Ec1multi rotEC(PC_10,PC_12,RESOLUTION);  //4逓倍用classエンコーダ1回転で4096,精密な制御が行いたいので


//モーター設定
MotorController motor_rot(PC_6,PC_8,DELTA_T, rotEC,rot_speed_pid,rot_omega_pid); //引数は下行
    //モーター正転、逆転、周期[s]、エンコーダ、速度制御用のPID、角度制御のPID

Ticker ticker;

void calcOmega();

int main(){
    ticker.attach(&calcOmega,0.01);//割り込み
    int count=0;
    double omega;
   
    while(1){
        count = rotEC.getCount();
        omega = rotEC.getOmega();
        pc.printf("%d, ",count);
        pc.printf("%lf\n",omega);
        wait(1);
    }
   
    motor_rot.Ac(30.0);
   
}

void calcOmega(){
    rotEC.calOmega();
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
    int theta
    wait(5);

    while 1{
        theta = Ec::getDeg()
        if (i<=0.5){
            i+=0.1
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
int servo(char option){
    servo.period_us(20000);  //周期設定
    while(1){
        servo.pulsewidth_us(500); //パルス幅変更 開いた状態
        wait(1);
        servo.pulsewidth_us(1500); //閉じた状態
        wait(3);
    }
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
        if(msg.id == 0){
            if (msg.data[3]==0){
                if (msg.data[0]==2){
                    if (msg.data[1] != 0){
                        status = move_arm(msg.data[1]);              
                    }
                    else if (msg.data[2] != 0){
                        int angle;
                        if (msg.data[2]==1){
                            angle = 40;
                        }
                        else if (msg.data[2]==2){
                            angle = 70;
                        }
                        else if (msg.data[2]==3){
                            angle = 100;
                        }
                        else {
                            alert();
                        }
                        status = rotate_arm(angle,angle+5)
                    }
                    else if (msg.data[3] != 0){
                        status = move_rack(msg.data[2]);
                    }

                    if (status == 0){
                        alert();
                    }
                }
            }
            else if (msg.data[3]==1){
                if (msg.data[1] != 0){
                    status = move_arm(msg.data[1]);             
                }
                else{
                    auto_arm(100);
                }
            }


        }
    }
}

int main(){
    move_arm(0);
    wait(10);
    ticker.attach(&control,0.001);
}
