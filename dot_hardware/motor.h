#include "pid.h"
#include <wiringPi.h>
#include <softPwm.h>
#include <pthread.h>

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

#define PI 3.14159
#define min(a,b) a>b?b:a
#define ENC_PULSE_PER_REV 2800

// This is for 150rpm motor with 100 gear ratio

/*
Mapping from PiGPIO to Wiringpi is given above

for more info run the command 
    lsgpio
    --or--
    gpio readall

Note:
    PWM range is 0 to 100 (not 255)
*/



class Motor{
    private:
        int motor_p;
        int motor_n;
        int motor_e;
        int m_effort;
        
    public:
        pthread_t thread_id;
        int motor_encA;
        int motor_encB;
        long pos;
        int inter_val;
        long set_target;

        PiD* pid_obj;
        Motor(int, int, int, int, int, double, double, double);
        void *control(void);
        double read();
        void set(long double);
        //void updateEncoder();
};
Motor::Motor(int p, int n, int e, int a, int b, double Kp, double Kd, double Ki):m_effort(100){
    motor_encA = a;
    motor_encB = b;
    motor_p = p;
    motor_n = n;
    motor_e = e;
    pos = 0;
    inter_val=0;
    pinMode(motor_p, OUTPUT);
    pinMode(motor_n, OUTPUT);
    softPwmCreate(motor_e, 0, 100);

    pid_obj = new PiD(Kp,Kd,Ki);
    pthread_create(&thread_id, NULL, &Motor::control, this);
}
double Motor::read(){
    return 2*PI*(double)((long double)pos/((long double)ENC_PULSE_PER_REV));
}

void* Motor::control(){
    double val = pid_obj->compute(this->pos);
    
    if(abs(val)<0.1)return;
    if(val<0){
        digitalWrite(motor_p, LOW);
        digitalWrite(motor_n, HIGH);
        softPwmWrite(motor_e, min(-val, m_effort));
    } else {
        digitalWrite(motor_p, HIGH);
        digitalWrite(motor_n, LOW);
        softPwmWrite(motor_e, min(val, m_effort));
    }
}

void Motor::set(long double target){
    set_target = target;
    long pos_in_int = ENC_PULSE_PER_REV*(long)(target/(2*PI));
    pid_obj->set(pos_in_int);
}