/**
 * @file manual_control.cpp
 * @brief CAN通信による制御信号を用いてアームを動作させるプログラム
 * @author Akihisa Watanabe
 * @date 2022.02.26
 */
#include <stdio.h>
#include "mbed.h"
#include "EC.h"
#include "SpeedController.h"
#include "CalPID.h"

CAN can1(p30,p29);
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

PwmOut arm_rotate_1(PB_6); 
PwmOut arm_rotate_2(PA_5);
arm_rotate_1.period(50);
arm_rotate_2.period(50);

PwmOut PWM_TRIGER(PB_6); //超音波センサモジュールのTriger端子に入力する信号
InterruptIn GET_PWM(PC_7); //割り込み入力端子の設定．マイコンから出力したPWM信号をD9端子から取り込む． 

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

// program to obtain the angle the motor moved
Ec::Ec(int res, int multi) : count_(0), pre_omega_(0), pre_count_(0), resolution_(res), multiplication_(multi)
{
    timer_.start();
    setGearRatio(1);
}
 
int Ec::getCount() const
{
    return count_;
}
 
double Ec::getRad() const
{
    return count_ * 2.0f * M_PI / (multiplication_ * resolution_ * gear_ratio_);
}
double Ec::getDeg() const
{
    return count_ * 2.0f * 180.0 / (multiplication_ * resolution_ * gear_ratio_);
}
void Ec::calOmega()
{
    double t = timer_.read();
    double delta_time = t - ptw_;
    acceleration_ = (pre_omega_ - pre2_omega_) / delta_time;
    pre2_omega_ = pre_omega_;
    pre_omega_ = omega_;
    omega_ = (count_ - pre_count_) * 2.0f * M_PI / (multiplication_ * resolution_ * delta_time);
    omega_ /= gear_ratio_;
    pre_count_ = count_;
    ptw_ = t;
}
 
double Ec::getOmega() const
{
    return omega_;
}
double Ec::getAcceleration() const
{
    return acceleration_;
}
void Ec::setResolution(int res)
{
    resolution_ = res;
}
 
/*reset関数の定義*/
/*エンコーダを初期状態に戻すことができる*/
void Ec::reset()
{
    count_ = 0;
    pre_count_ = 0, omega_ = 0;
    ptw_ = 0;
    timer_.stop();
    timer_.reset();
    timer_.start();
}
void Ec::setGearRatio(double gear_r)
{
    gear_ratio_ = gear_r;
}
 
////////////////////////////////////////////////////1逓倍//////////////////////////////////////////////////////////////////
Ec1multi::Ec1multi(PinName signalA, PinName signalB, int res) : Ec(res, 1), signalA_(signalA), signalB_(signalB)
{
    signalA_.rise(callback(this, &Ec1multi::upA));
}
 
//ピン変化割り込み関数の定義
void Ec1multi::upA()
{
    if (signalB_.read())
        count_++;
    else
        count_--;
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
    //ここにアームを上げる処理．
    wait(10);
    ticker.attach(&control,0.001);
}
