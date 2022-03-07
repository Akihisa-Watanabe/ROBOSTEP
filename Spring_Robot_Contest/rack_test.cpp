#include "mbed.h"

Ticker ticker;

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
    int i=0;
    switch (option){
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

}

/**
 * @fn
 * @brief CAN通信で制御信号を受信してアームを制御する関数
 */
void control(){
    print_result(move_rack(0));
    print_result(move_rack(1));
}

int main(){
    rack_pull.period(0.05);
    rack_push.period(0.05);
    wait(10);
    ticker.attach(&control,0.001);
}
