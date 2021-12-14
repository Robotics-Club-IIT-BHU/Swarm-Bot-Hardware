#include <unistd.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Imu.h>
#include <cstdlib>

#define L 0.04
#define R 0.01905
#define piby30 0.1047197551196597705355242034774843062905347323976457118988037109375 // long double thirty = 30; long double mOne = -1; printf("%1.70Lf\n", (long double) acos(mOne) / thirty);
#define piby2 1.5707963267948965579989817342720925807952880859375 // long double two = 2; long double mOne = -1; printf("%1.70Lf\n", (long double) acos(mOne) / two);
#define pi 3.141592653589793115997963468544185161590576171875 // long double mOne = -1; printf("%1.70Lf\n", (long double) acos(mOne));
#define pi2 6.28318530717958623199592693708837032318115234375 // long double two = 2; long double mOne = -1; printf("%1.70Lf\n", (long double) two * acos(mOne));
#define sqrt3 1.732050807568877193176604123436845839023590087890625 // long double three = 3; printf("%1.70Lf\n", (long double) sqrt(three));
#define sqrt3by2 0.8660254037844385965883020617184229195117950439453125 // long double two = 2; long double three = 3; printf("%1.70Lf\n", (sqrt(three) / two));

#define L_IND 0
#define R_IND 1
#define B_IND 2


ros::Time timeCurrent;
ros::Time timePrevious;
ros::Publisher pub_;
ros::Subscriber imu_sub, cmd_vel_sub, jnt_sub;
ros::Publisher l_pub_, r_pub_, b_pub_;

bool imu_flag=true;
double yaw_offset = 0;
double vx=0;
double vy=0;
double wp=0;


struct Wheel{
    double l_theta;
    double l_theta_dot;
    double r_theta;
    double r_theta_dot;
    double b_theta;
    double b_theta_dot;
} wheel_;

struct Pose{
    double x;
    double y;
    double rol;
    double pit;
    double yaw;
} p_;

struct Odom{
    double x;
    double y;
    double theta;
    double vx;
    double vy;
    double wx;
} odom_;

void updateAndOdom(Wheel wheel);
void publishOdom();

void jnt_state_callback(const sensor_msgs::JointState &msg){
    double jnt_vel;
    for(int i=0;i<3;i++){
        if(msg.name[i][0]=='l'){
            wheel_.l_theta_dot = msg.velocity[i];
            wheel_.l_theta = msg.position[i];
        } else if (msg.name[i][0]=='r'){
            wheel_.r_theta_dot = msg.velocity[i];
            wheel_.r_theta = msg.position[i];
        } else if (msg.name[i][0]=='b'){
            wheel_.b_theta_dot = msg.velocity[i];
            wheel_.b_theta = msg.position[i];
        }
    }
    updateAndOdom(wheel_);
    
}

void updateAndOdom(Wheel wheel){
    timeCurrent = ros::Time::now();
    double duration, v_left, v_back, v_right;
    duration = (timeCurrent - timePrevious).toSec();
    
    v_left = wheel.l_theta_dot;
    v_right = wheel.r_theta_dot;
    v_back = wheel.b_theta_dot;

    timePrevious = timeCurrent;

    long double v_left0  = v_left  * R;
    long double v_back0  = v_back  * R;
    long double v_right0 = v_right * R;
    double x,y,theta;
    y     = ((2.0 * v_back0) - v_left0 - v_right0) / 3.0;
    x     = ((1.73 * v_right0) - (1.73 * v_left0)) / 3.0;
    // theta = (v_left0 + v_back0 + v_right0) / (3*0.04);

    double X = cos(odom_.theta)*x - sin(odom_.theta)*y;
    double Y = sin(odom_.theta)*x + cos(odom_.theta)*y;

    // duration = (timeCurrent - timePrevious).toSec();
    odom_.vx = X;
    odom_.vy = Y;
    odom_.x += X * duration;
    odom_.y += Y * duration;
    // odom_.theta += theta * duration;

    publishOdom();
}
void publishOdom(){
    static tf::TransformBroadcaster br;
    geometry_msgs::TransformStamped odom_trans;

    tf::Quaternion tf_quat;
    tf_quat.setRPY(0, 0, odom_.theta);

    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "origin_link";
    odom_trans.transform.translation.x =odom_.x;
    odom_trans.transform.translation.y =odom_.y;
    odom_trans.transform.translation.z =0;

    geometry_msgs::Quaternion geo_Quat ;
    geo_Quat.x = tf_quat.x();
    geo_Quat.y = tf_quat.y();
    geo_Quat.z = tf_quat.z();
    geo_Quat.w = tf_quat.w();
    odom_trans.transform.rotation = geo_Quat ;

    br.sendTransform(odom_trans);

    //publish odometry over ros
    nav_msgs::Odometry odom ;
    odom.header.stamp = odom_trans.header.stamp ;
    odom.header.frame_id = "odom" ;

    //set the position
    // std::cout<<odom_trans.transform.translation.x<<" "
    //   <<odom_trans.transform.translation.y<<" "
    //   <<odom_trans.transform.translation.z<<"\n";
    odom.pose.pose.position.x = odom_trans.transform.translation.x ;
    odom.pose.pose.position.y = odom_trans.transform.translation.y ;
    odom.pose.pose.position.z = odom_trans.transform.translation.z ;
    odom.pose.pose.orientation = geo_Quat ;

    //set the velocity
    odom.child_frame_id = "origin_link";
    odom.twist.twist.linear.x = odom_.vx ;
    odom.twist.twist.linear.y = odom_.vy ;
    odom.twist.twist.linear.z = 0 ;

    odom.twist.twist.angular.x= 0 ;
    odom.twist.twist.angular.y= 0 ;
    odom.twist.twist.angular.z= odom_.theta ;

    pub_.publish(odom);
}
void velocity_callback(const geometry_msgs::Twist& msg){
    vx = msg.linear.x;
    vy = msg.linear.y;
    wp = msg.angular.z;
    // std::cout<<vx<<" "<<vy<<" "<<wp<<" "<<"\n";
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg){
    tf::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(p_.rol, p_.pit, p_.yaw);
    odom_.wx = msg->angular_velocity.z;

    if(imu_flag){
        imu_flag=false;
        yaw_offset = p_.yaw;
    }

    odom_.theta = p_.yaw - yaw_offset;
}

// class OmniDriver{
//     private:

//     public:
//         double l_w_dir=1, b_w_dir=1, r_w_dir=1;
//         OmniDriver(ros::NodeHandle* n){

//         }
// };

int main(int argc, char** argv){

    ros::init(argc, argv, "omnidrive");
    ros::NodeHandle n("");
    timePrevious = ros::Time::now();
    int debug = getenv("DEBUG")?atoi(getenv("DEBUG")):0;
    // double wheel_speed = getenv("WSP")?atoi(getenv("WSP")):10;
    double hz=100;
    ros::Rate rate(hz);
    double dt_ = 1.0/hz;
    // OmniDriver* div;
    // div = new OmniDriver(&n);

    cmd_vel_sub = n.subscribe("cmd_vel", 1000, velocity_callback);
    jnt_sub = n.subscribe("joint_state", 1000, jnt_state_callback);
    imu_sub = n.subscribe("imu/data", 100, imu_callback); // use the one with madwigk filter not this
    pub_ = n.advertise<nav_msgs::Odometry>("odom", 50) ;
    l_pub_ = n.advertise<std_msgs::Float64>("velocity_controller/left_joint_vel_controller/command", 10);
    r_pub_ = n.advertise<std_msgs::Float64>("velocity_controller/right_joint_vel_controller/command", 10);
    b_pub_ = n.advertise<std_msgs::Float64>("velocity_controller/back_joint_vel_controller/command", 10);
    long int i = 0;

    while(ros::ok()){
       
        
        double vmx= vx;
        double vmy= vy;
        double wmp = wp ; // Body frame
        
        double v1, v2, v3;
        v1 = (L * wmp - (vmx / 2) - (sqrt3by2 * vmy));
        v2 = (vmx + L * wmp);
        v3 = (L * wmp - (vmx / 2) + (sqrt3by2 * vmy));

        //v1=v2=v3=1;
        // wheel_.lpos = wheel_.lpos + div->l_w_dir*wheel_speed*v1*dt_/R;
        // wheel_.bpos = wheel_.bpos + div->b_w_dir*wheel_speed*v2*dt_/R;
        // wheel_.rpos = wheel_.rpos + div->r_w_dir*wheel_speed*v3*dt_/R;
        std_msgs::Float64 wheel_vel;
        wheel_vel.data = v1;
        l_pub_.publish(wheel_vel);
        wheel_vel.data = v2;
        b_pub_.publish(wheel_vel);
        wheel_vel.data = v3;
        r_pub_.publish(wheel_vel);

        publishOdom();

        ros::spinOnce();
        rate.sleep();
        
    }
    return 0;
}