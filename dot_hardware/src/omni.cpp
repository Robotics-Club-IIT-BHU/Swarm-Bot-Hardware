#include "dot_hardware/motor.h"
#include <wiringPi.h>
#include <unistd.h>
#include <ros/ros.h>


#define L 
#define R 0.01905

#define L_IND 0
#define R_IND 1
#define B_IND 2
Motor* l;
Motor* r;
Motor* b;

ros::Time timeCurrent;
ros::Time timePrevious;
ros::Publisher pub_;

double vx;
double vy;
double wp;

struct Wheel{
    double lpos;
    double rpos;
    double bpos;
} wheel_;

struct Pose{
    double x;
    double y;
    double r;
    double p;
    double y;
} p_;

struct Odom{
    double x;
    double y;
    double theta;
} odom_;

void updateAndOdom(double* read){
    timeCurrent = ros::Time::now();
    double duration, v_left, v_back, v_right;
    duration = (timeCurrent - timePrevious).toSec();
    v_left  = (read[L_IND]  - wheel_.lpos ) / duration;
    v_back  = (read[B_IND]  - wheel_.bpos ) / duration;
    v_right = (read[R_IND]  - wheel_.rpos) / duration;
    timePrevious = timeCurrent;

    long double v_left0  = v_left  * R;
    long double v_back0  = v_back  * R;
    long double v_right0 = v_right * R;
    double x,y,theta;
    y     = ((2.0 * v_back0) - v_left0 - v_right0) / 3.0;
    x     = ((1.73 * v_right0) - (1.73 * v_left0)) / 3.0;
    theta = (v_left0 + v_back0 + v_right0) / (3*0.04);

    double X = cos(odom_theta)*x + sin(odom_theta)*y;
    double Y = sin(odom_theta)*x - cos(odom_theta)*y;

    odom_.x += X * duration;
    odom_.y += Y * duration;
    odom_.theta += theta * duration;


}
void publishOdom(){
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped odom_trans;

    tf2::Quaternion tf_quat;
    tf_quat.setRPY(0, 0, odom_theta);

    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "origin_link";
    odom_trans.transform.translation.x =odom_x;
    odom_trans.transform.translation.y =odom_y;
    odom_trans.transform.translation.z =0;

    geometry_msgs::Quaternion geo_Quat ;
    geo_Quat=tf2::toMsg(tf_quat);
    odom_trans.transform.rotation = geo_Quat ;

    br.sendTransform(odom_trans);

    //publish odometry over ros
    nav_msgs::Odometry odom ;
    odom.header.stamp = odom_trans.header.stamp ;
    odom.header.frame_id = "odom" ;

    //set the position
    cout<<odom_trans.transform.translation.x<<" "
      <<odom_trans.transform.translation.y<<" "
      <<odom_trans.transform.translation.z<<"\n";
    odom.pose.pose.position.x = odom_trans.transform.translation.x ;
    odom.pose.pose.position.y = odom_trans.transform.translation.y ;
    odom.pose.pose.position.z = odom_trans.transform.translation.z ;
    odom.pose.pose.orientation = geo_Quat ;

    //set the velocity
    odom.child_frame_id = "origin_link";
    odom.twist.twist.linear.x = x ;
    odom.twist.twist.linear.y = y ;
    odom.twist.twist.linear.z = 0 ;

    odom.twist.twist.angular.x= 0 ;
    odom.twist.twist.angular.y= 0 ;
    odom.twist.twist.angular.z= theta ;

    pub_.publish(odom);
}
void velocity_callback(const geometry_msgs::Twist& msg){
    vx = msg.linear.x;
    vy = msg.linear.y;
    wp = msg.angular.z;
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg){
    tf::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
    tf::Matrix3x3 m(q);

    m.getRPY(p_.r, p_.p, p_.y);
    odom_theta = p_.y;
}


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

int main(int argc, char** argv){

    ros::init(argc, argv, "omnidrive");
    ros::NodeHandle n("");
    timePrevious = ros::Time::now();

    double hz=100;
    ros::Rate rate(hz);
    
    OmniDriver* div;
    div = new OmniDriver();

    cmd_vel_sub = n.subscribe("cmd_vel", 1000, velocity_callback, this);
    imu_sub = n.subscribe("imu", 100, imu_callback);
    pub_ = n.advertise<nav_msgs::Odometry>("odom", 50) ;
    
    while(rps::ok()){
        double read[3];
        div->readings(read);
        updateAndOdom(read);
        
        //std::cout<<"L: "<<read[L_IND]<<" R: "<<read[R_IND]<<" B: "<<read[B_IND]<<"\n";
        wheel_.lpos = read[L_IND];
        wheel_.rpos = read[R_IND];
        wheel_.bpos = read[B_IND];
        //delay(10);
        
        double vmx=cos(yaw)*vx-sin(yaw)*vy;
        double vmy=-sin(yaw)*vx-cos(yaw)*vy;
        double wmp = wp ;//- yaw;
        
        double v1, v2, v3;
        v1 = (L * wmp - (vmx / 2) - (sqrt3by2 * vmy));
        v2 = (vmx + L * wmp);
        v3 = (L * wmp - (vmx / 2) + (sqrt3by2 * vmy));

    
        wheel_.lpos = wheel_.lpos + 10*v1*dt_/r;
        wheel_.bpos = wheel_.bpos + 10*v2*dt_/r;
        wheel_.rpos = wheel_.rpos + 10*v3*dt_/r;

        l->set(wheel_.lpos);
        r->set(wheel_.rpos);
        b->set(wheel_.bpos);

        publishOdom();

        ros::spinOnce();
        rate.sleep();
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