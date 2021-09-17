#include "pid.h"
#include <wiringPi.h>
#include <softPWM.h>

#define pi_0 30
#define pi_1 31
#define pi_2 8
#define pi_3 9
#define pi_4 7
#define pi_5 21
#define pi_6 22
#define pi_7 11
#define pi_8 10
#define pi_9 13
#define pi_10 12
#define pi_11 14
#define pi_12 26
#define pi_13 23
#define pi_14 15
#define pi_15 16
#define pi_16 27
#define pi_17 0
#define pi_18 1
#define pi_19 24
#define pi_20 28
#define pi_21 29
#define pi_22 3
#define pi_23 4
#define pi_24 5
#define pi_25 6
#define pi_26 25
#define pi_27 2

/*
Mapping from PiGPIO to Wiringpi is given above

for more info run the command 
    lsgpio
    --or--
    gpio readall

Note:
    PWM range is 0 to 100 (not 255)
*/


int main(){
    wiringPiSetup();
    pinMode(pi_22, OUTPUT);
    pinMode(pi_23, OUTPUT);
    softPwmCreate(pi_27, 0, 100);
    count = 0;
    while(true){
        softPwmWrite(pi_27, 100);
        if((count/10000)%2){
            digitalWrite(pi_22, HIGH);
            digitalWrite(pi_23, LOW);
        } else {
            digitalWrite(pi_22, LOW);
            digitalWrite(pi_23, HIGH);
        }
        count++;
    }
    return 0;
}