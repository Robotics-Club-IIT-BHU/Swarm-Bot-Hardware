#include <stdio.h>
#include <wiringPi.h>
#include <softServo.h>
#include "utils.h"

int get_signal(double angle){
    return 1500*(angle/PI) -250;
}

int main(){
    wiringPiSetup();
    softServoSetup(pi_14);
    val = 0;
    d = 0.01;
    while(val<0.9){
        softServoWrite(pi_14, val);
        val += d;
    }
    return 0;
    
}

