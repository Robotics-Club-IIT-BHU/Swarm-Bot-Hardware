#include <iostream>
#include <chrono>
#define max(a,b) a>b?a:b

class PiD{
    using clock_t = std::chrono::high_resolution_clock;
    private:
        double dt_;
        double Kp;
        double Kd;
        double Ki;
        double error;
        double prev_error;
        double error_i;
        long double target;
        std::chrono::high_resolution_clock::time_point prev;

    public:
        PiD(double, double, double, double);
        void tune(double, double, double, double);
        double compute(long double);
        void set(long double);
        void reset(void);
};

PiD::PiD(double kp=-1, double kd=-1, double ki=-1, double dt=-1):Kp(10), Kd(100), Ki(0.2)
{
    if(kp!=-1)Kp=kp;
    if(kd!=-1)Kd=kd;
    if(ki!=-1)Ki=ki;
    if(dt!=-1)dt_=dt;

    prev_error=0;
    error = 0;
    error_i = 0;
    prev = clock_t::now();
}
void PiD::tune(double kp=-1, double kd=-1, double ki=-1, double dt=-1){

    if(kp!=-1)Kp=kp;
    if(kd!=-1)Kd=kd;
    if(ki!=-1)Ki=ki;
    if(dt!=-1)dt_=dt;

    
}
double PiD::compute(long double current){
    double pid_feed=0;
    std::chrono::high_resolution_clock::time_point curr = clock_t::now();
    dt_ = std::chrono::duration<double>(curr - prev).count();
    dt_ = max(dt_, 0.0001);
    if(error==prev_error){
        error = target-current;
        pid_feed = Kp*error;
        prev_error = error; 
    } else {
        error = target-current;
        if(error<0.1) error_i+=error;
        else error_i=0;
        
        pid_feed = Kp*error + Kd*((error-prev_error)/dt_) + Ki*error_i;
        prev_error = error;
    }
    prev = curr;

}