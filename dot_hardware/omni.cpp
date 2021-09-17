#include "motor.h"
#include <wiringPi.h>

#define L_IND 0
#define R_IND 1
#define B_IND 2
Motor* l;
Motor* r;
Motor* b;

void updateEncoderL(){    
    int MSB = digitalRead(l->motor_encA);
    int LSB = digitalRead(l->motor_encB);

    int encoded = (MSB<<1)|LSB;
    int sum = (l->inter_val<<2)|encoded;
    
    if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
        l->pos--;
    if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
        l->pos++;

    l->inter_val = encoded;
}

void updateEncoderB(){    
    int MSB = digitalRead(b->motor_encA);
    int LSB = digitalRead(b->motor_encB);

    int encoded = (MSB<<1)|LSB;
    int sum = (b->inter_val<<2)|encoded;
    
    if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
        b->pos--;
    if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
        b->pos++;

    b->inter_val = encoded;
}

void updateEncoderR(){    
    int MSB = digitalRead(r->motor_encA);
    int LSB = digitalRead(r->motor_encB);

    int encoded = (MSB<<1)|LSB;
    int sum = (r->inter_val<<2)|encoded;
    
    if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
        r->pos--;
    if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
        r->pos++;

    r->inter_val = encoded;
}


class OmniDriver{
    private:

    public:
        OmniDriver(){

#ifndef __PI_WIRING_SET__
#define __PI_WIRING_SET__
            wiringPiSetup();
#endif

            l = new Motor(pi_22, pi_23, pi_27, pi_17, pi_4, 10.0, 100.0, 0.0);
            r = new Motor(pi_10, pi_9, pi_25, pi_20, pi_21, 10.0, 100.0, 0.0);
            b = new Motor(pi_6, pi_5, pi_12, pi_11, pi_26, 10.0, 100.0, 0.0);
            
            wiringPiISR(l->motor_encA, INT_EDGE_BOTH, updateEncoderL);
            wiringPiISR(l->motor_encB, INT_EDGE_BOTH, updateEncoderL);

            wiringPiISR(r->motor_encA, INT_EDGE_BOTH, updateEncoderR);
            wiringPiISR(r->motor_encB, INT_EDGE_BOTH, updateEncoderR);

            wiringPiISR(b->motor_encA, INT_EDGE_BOTH, updateEncoderB);
            wiringPiISR(b->motor_encB, INT_EDGE_BOTH, updateEncoderB);
        }
        
        void readings(double* _re){
            _re[L_IND] = l->read();
            _re[R_IND] = r->read();
            _re[B_IND] = b->read();
            return _re;
        }
};

int main(){

    OmniDriver div;
    while(true){
        double read[3];
        div->readings(read);
        std::cout<<"L: "<<read[L_IND]<<" R: "<<read[R_IND]<<" B: "<<read[B_IND]<<"\n";
        delay(10);
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