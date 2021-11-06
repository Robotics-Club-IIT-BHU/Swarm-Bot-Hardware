#include "pid.h"

#define PI 3.14159
#define INT_MAX 9999999
#define ENC_PULSE_PER_REV 2800

#define LMOTOR_ENCA PA0
#define LMOTOR_ENCB PA1
#define RMOTOR_ENCA PA2
#define RMOTOR_ENCB PA3
#define BMOTOR_ENCA PA4
#define BMOTOR_ENCB PA5

#define LMOTOR_P PB3
#define LMOTOR_N PB4
#define LMOTOR_M PB5
#define RMOTOR_P PB6
#define RMOTOR_N PB7
#define RMOTOR_M PB8
#define BMOTOR_P PB2
#define BMOTOR_N PB1
#define BMOTOR_M PB0


class Motor{
    private:
        double m_effort;
        double m_vel; // 200 rpm motors are used hence it would be 200*2*pi/60
        
    public:
        // pthread_t thread_id;
        int motor_p;
        int motor_n;
        int motor_e;
        int motor_encA;
        int motor_encB;
        long pos;
        long prev_pos;
        double curr_vel;
        double prev_inp;
        int inter_val;
        long set_target;
        long unsigned int prev_check;

        PiD* pid_obj;
        Motor(int, int, int, int, int, double, double, double);
        ~Motor();
        int control(void);
        double read(long);
        void setPos(long double);
        void setVel(long double);
        void clear();
        //void updateEncoder();
};
Motor::Motor(int p, int n, int e, int a, int b, double Kp, double Kd, double Ki):m_effort(65000), m_vel(21.0){
    motor_encA = a;
    motor_encB = b;
    motor_p = p;
    motor_n = n;
    motor_e = e;
    pos = 0;
    prev_pos = 0;
    curr_vel = 0;
    inter_val=0;
    prev_inp = 0;
//    pinMode(motor_p, OUTPUT);
//    pinMode(motor_n, OUTPUT);
    // softPwmCreate(motor_e, 0, 100);
//    pinMode(motor_e, OUTPUT);
    pid_obj = new PiD(Kp,Kd,Ki);
}
double Motor::read(long val=-INT_MAX){
    if(val==-INT_MAX)
        return 2*PI*(double)((long double)pos/((long double)ENC_PULSE_PER_REV));
    else
        return 2*PI*(double)((long double)val/((long double)ENC_PULSE_PER_REV));
}

int Motor::control(){
    double val = 257*pid_obj->compute(this->curr_vel);
    if(abs(val)<257*5) return -1;
    val += prev_inp;
    prev_inp = val;
    //std::cout<<val<<"\n";  
    if(abs(val)<257*20){
        digitalWrite(motor_p, LOW);
        digitalWrite(motor_n, LOW);
        analogWrite(motor_e, 0);
        return 0;
    }
    if(val<0){
        digitalWrite(motor_p, HIGH);
        digitalWrite(motor_n, LOW);
        analogWrite(motor_e, (int)min(-val, m_effort)); //softPwmWrite(motor_e, (int)min(-val, m_effort));
    } else {
        digitalWrite(motor_p, LOW);
        digitalWrite(motor_n, HIGH);
        analogWrite(motor_e, (int)min( val, m_effort)); //softPwmWrite(motor_e, (int)min(val, m_effort));
    }
    return 1;
}

void Motor::setPos(long double target){
    set_target = target;
    long pos_in_int = ENC_PULSE_PER_REV*(long)(target/(2*PI));
    pid_obj->set(pos_in_int);
}
void Motor::setVel(long double target){
    target = min(target, m_vel);
    target = max(target, -m_vel);
    set_target = target;

    pid_obj->set(set_target);
}
Motor::~Motor(){
    digitalWrite(motor_p, LOW);
    digitalWrite(motor_n, LOW);
    // softPwmWrite(motor_e, 0);   
    analogWrite(motor_e, 0);
}
void Motor::clear(){
    digitalWrite(motor_p, LOW);
    digitalWrite(motor_n, LOW);
    // softPwmWrite(motor_e, 0);   
    analogWrite(motor_e, 0);
}
