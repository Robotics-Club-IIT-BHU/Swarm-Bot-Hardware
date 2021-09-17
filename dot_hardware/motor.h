#include "pid.h"
#include <wiringPi.h>
#include <softPwm.h>
#include <pthread.h>
#include "utils.h"

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
        int control(void);
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
}
double Motor::read(){
    return 2*PI*(double)((long double)pos/((long double)ENC_PULSE_PER_REV));
}

int Motor::control(){
    double val = pid_obj->compute(this->pos);
    //std::cout<<val<<"\n";  
    if(abs(val)<5)return 0;
    if(val<0){
        digitalWrite(motor_p, HIGH);
        digitalWrite(motor_n, LOW);
        softPwmWrite(motor_e, (int)min(-val, m_effort));
    } else {
        digitalWrite(motor_p, LOW);
        digitalWrite(motor_n, HIGH);
        softPwmWrite(motor_e, (int)min(val, m_effort));
    }
    return 1;
}

void Motor::set(long double target){
    set_target = target;
    long pos_in_int = ENC_PULSE_PER_REV*(long)(target/(2*PI));
    pid_obj->set(pos_in_int);
}