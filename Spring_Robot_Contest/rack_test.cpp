#include "mbed.h"

Ticker ticker;
Timer ActiveTime; //タイマー計測用変数

int move_rack(char option=0);
void control();

Serial pc(USBTX,USBRX);



PwmOut rack_pull(PC_8); 
PwmOut rack_push(PC_6);

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
void print_result(int status){
    /*
    if (status==0){
        pc.printf("Success.\n");
    }
    else{
        pc.printf("Error.\n");
    }*/
}

/**
 * @fn
 * @brief CAN通信で制御信号を受信してアームを制御する関数
 */
void control(){
        //int status;//,angle=0;
    /*
    printf("Arm up-down test.\n");
    print_result(move_arm(0));  
    print_result(move_arm(1));  

    printf("Arm rotate test.\n");
    angle=0;
    print_result(rotate_arm(angle,angle+5));
    angle=40;
    print_result(rotate_arm(angle,angle+5));
    */
    //printf("Rack move test.\n");
    print_result(move_rack(0));
    print_result(move_rack(1));

    //printf("Ultrasonic Sensor test.\n")
    //auto_arm(100);
}

int main(){
    //arm_down.period(0.05);
    //arm_up.period(0.05);
    //arm_rotate_1.period(0.05);
    //arm_rotate_2.period(0.05);
    rack_pull.period(0.05);
    rack_push.period(0.05);
    //move_arm(0);
    wait(10);
    ticker.attach(&control,0.001);
}
