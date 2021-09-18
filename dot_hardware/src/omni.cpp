#include "motor.h"
#include <wiringPi.h>
#include <unistd.h>

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

void* controlL(void *vargp)
{
    //std::cout<<"lcontrol\n";
    while(true){
        l->control();
        usleep(10);
    }
}
void* controlR(void *vargp)
{
    //std::cout<<"rcontrol\n";
    while(true){
        r->control();
        usleep(10);
    }
    
}
void* controlB(void *vargp)
{
    //std::cout<<"bcontrol\n";
    while(true){
        b->control();
        usleep(10);
    }
}
class OmniDriver{
    private:

    public:
        OmniDriver(){

#ifndef __PI_WIRING_SET__
#define __PI_WIRING_SET__
            wiringPiSetup();
#endif

            l = new Motor(pi_23, pi_22, pi_27, pi_17, pi_4, 5.0, 1.0, 0.01);
            r = new Motor(pi_10, pi_9, pi_25, pi_20, pi_21, 5.0, 1.0, 0.01);
            b = new Motor(pi_6, pi_5, pi_12, pi_11, pi_26, 5.0, 1.0, 0.01);
            
            wiringPiISR(l->motor_encA, INT_EDGE_BOTH, updateEncoderL);
            wiringPiISR(l->motor_encB, INT_EDGE_BOTH, updateEncoderL);

            wiringPiISR(r->motor_encA, INT_EDGE_BOTH, updateEncoderR);
            wiringPiISR(r->motor_encB, INT_EDGE_BOTH, updateEncoderR);

            wiringPiISR(b->motor_encA, INT_EDGE_BOTH, updateEncoderB);
            wiringPiISR(b->motor_encB, INT_EDGE_BOTH, updateEncoderB);
            //std::cout<<"here";
            pthread_create(&(l->thread_id), NULL, controlL, NULL);
            pthread_create(&(r->thread_id), NULL, controlR, NULL);
            pthread_create(&(b->thread_id), NULL, controlB, NULL);
        }
        
        void readings(double* _re){
            _re[L_IND] = l->read();
            _re[R_IND] = r->read();
            _re[B_IND] = b->read();
        }
};

int main(){

    OmniDriver* div;
    div = new OmniDriver();
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