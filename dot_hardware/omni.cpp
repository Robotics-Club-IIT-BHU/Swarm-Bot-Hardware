#include "motor.h"
#include <wiringPi.h>

Motor* a;

void updateEncoderA(){
    int MSB = digitalRead(a->motor_encA);
    int LSB = digitalRead(a->motor_encB);

    int encoded = (MSB<<1)|LSB;
    int sum = (inter_val<<2)|encoded;
    
    if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
        pos--;
    if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
        pos++;

    inter_val = encoded;
}
int main(){

    a = new Motor(pi_22, pi_23, pi_27, pi_17, pi_18, 10.0, 100.0, 0.0);
    wiringPiISR(a->motor_encA, INT_EDGE_BOTH, updateEncoderA);
    wiringPiISR(a->motor_encB, INT_EDGE_BOTH, updateEncoderA);
    while(true){
        std::cout<<a->read();
        delay(100);
        // softPwmWrite(pi_27, 100);
        // if((count/10000)%2){
        //     digitalWrite(pi_22, HIGH);
        //     digitalWrite(pi_23, LOW);
        // } else {
        //     digitalWrite(pi_22, LOW);
        //     digitalWrite(pi_23, HIGH);
        // }
        // count++;
    }
    return 0;
}