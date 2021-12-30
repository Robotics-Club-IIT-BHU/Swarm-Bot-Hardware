/*
* file: omni.cpp
* author: Somnath Sendhil Kumar
* data: 5th Dec 2021
*/

#include <stdio.h>
#include <iostream>
#include <atomic>
#include <stdlib.h>
#include <string>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/signal.h>
#include <errno.h>
#include <termios.h>
#include <cstdlib>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Empty.h>

#define BUF_SIZE 256

#include <unistd.h>
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

// ros::Publisher jnt_state_pub_;
ros::Publisher odom_pub_;
ros::Subscriber cmd_vel_sub_;
ros::Subscriber stm_reset_sub_;
sensor_msgs::JointState jnt_st;

struct ttas_lock {
  std::atomic<bool> lock_ = {false};

  void unlock() {lock_.store(false);}

  void lock() {
    for (;;) {
      if (!lock_.exchange(true, std::memory_order_acquire)) {
        break;
      }
      while (lock_.load(std::memory_order_relaxed));
    }
  }
};


void signal_handler_IO (int status);   /* definition of signal handler */
void inp_parse(int res);
void lf_wheel_callback(const std_msgs::Float64& msg);
void rt_wheel_callback(const std_msgs::Float64& msg);
void bk_wheel_callback(const std_msgs::Float64& msg);

void stm_reset(const std_msgs::Empty& msg);

struct ttas_lock write_lock;
int n;
int fd;
int connected;
struct termios termAttr;
struct sigaction saio;
char buf[BUF_SIZE];
bool start_omni = false;

bool imu_flag=true;
double yaw_offset = 0;
double vx=0;
double vy=0;
double wp=0;
std::string device_name_;

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
    double wz;
} odom_;

void updateAndOdom(Wheel wheel);
void publishOdom();

void jnt_state_callback(sensor_msgs::JointState msg){
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
    start_omni = true;
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

    long double v_left0  = -v_left  * R;
    long double v_back0  = -v_back  * R;
    long double v_right0 = -v_right * R;
    double x,y,theta;
    x     = ((2.0 * v_back0) - v_left0 - v_right0) / 3.0;
    y     = ((1.73 * v_right0) - (1.73 * v_left0)) / 3.0;
    // theta = (v_left0 + v_back0 + v_right0) / (3*0.04);

    double X = cos(odom_.theta)*x - sin(odom_.theta)*y;
    double Y = sin(odom_.theta)*x + cos(odom_.theta)*y;

    // duration = (timeCurrent - timePrevious).toSec();
    odom_.vx = X;
    odom_.vy = Y;
    odom_.wz = theta;
    odom_.x += X * duration;
    odom_.y += Y * duration;
    odom_.theta += theta * duration;

    //publishOdom();
}
void publishOdom(){
    static tf::TransformBroadcaster br;
    geometry_msgs::TransformStamped odom_trans;

    tf::Quaternion tf_quat;
    tf_quat.setRPY(0, 0, odom_.theta);

    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = device_name_+"/odom";
    odom_trans.child_frame_id = device_name_+"/base_link";
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
    odom.header.frame_id = device_name_+"/odom" ;

    //set the position
    // std::cout<<odom_trans.transform.translation.x<<" "
    //   <<odom_trans.transform.translation.y<<" "
    //   <<odom_trans.transform.translation.z<<"\n";
    odom.pose.pose.position.x = odom_trans.transform.translation.x ;
    odom.pose.pose.position.y = odom_trans.transform.translation.y ;
    odom.pose.pose.position.z = odom_trans.transform.translation.z ;
    odom.pose.pose.orientation = geo_Quat ;

    //set the velocity
    odom.child_frame_id = device_name_+"/base_link";
    odom.twist.twist.linear.x = odom_.vx ;
    odom.twist.twist.linear.y = odom_.vy ;
    odom.twist.twist.linear.z = 0 ;

    odom.twist.twist.angular.x= 0 ;
    odom.twist.twist.angular.y= 0 ;
    odom.twist.twist.angular.z= odom_.wz ;

    odom_pub_.publish(odom);
}
void velocity_callback(const geometry_msgs::Twist& msg){
    vx = msg.linear.x;
    vy = msg.linear.y;
    // wp = msg.angular.z;
    // std::cout<<vx<<" "<<vy<<" "<<wp<<" "<<"\n";
}
void angular_callback(const geometry_msgs::Twist& msg){
    wp = msg.angular.z;
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg){
    tf::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(p_.rol, p_.pit, p_.yaw);
    odom_.wz = msg->angular_velocity.z;

    if(imu_flag){
        imu_flag=false;
        yaw_offset = p_.yaw;
    }

    odom_.theta = p_.yaw - yaw_offset;
}


int main(int argc, char** argv){

    ros::init(argc, argv, "omnidrive");
    ros::NodeHandle n("");
    timePrevious = ros::Time::now();
    cmd_vel_sub_ = n.subscribe("cmd_vel", 3, velocity_callback);
    //imu_sub = n.subscribe("imu/data", 100, imu_callback); // use the one with madwigk filter not this
    odom_pub_ = n.advertise<nav_msgs::Odometry>("odom", 50) ;
    long int i = 0;
    // jnt_state_pub_ = n.advertise<sensor_msgs::JointState>("joint_state",100);
    device_name_ = getenv("ROS_DEVICE_NAME");
    jnt_st.header.frame_id = device_name_+"/base_link";
    jnt_st.header.stamp = ros::Time::now();
    jnt_st.name = std::vector<std::string>(3,"none");
    jnt_st.name[0] = "left_joint";
    jnt_st.name[2] = "right_joint";
    jnt_st.name[1] = "back_joint";

    jnt_st.position = std::vector<double> (3,0);
    jnt_st.velocity = std::vector<double> (3,0);
    jnt_st.effort = std::vector<double> (3,0);

    fd = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1)
    {
       perror("open_port: Unable to open /dev/ttyAMA0\n");
       exit(1);
    }

    saio.sa_handler = signal_handler_IO;
    saio.sa_flags = 0;
    saio.sa_restorer = NULL; 
    sigaction(SIGIO,&saio,NULL);

    fcntl(fd, F_SETFL, FNDELAY);
    fcntl(fd, F_SETOWN, getpid());
    fcntl(fd, F_SETFL,  O_ASYNC ); /**<<<<<<------This line made it work.**/

    tcgetattr(fd,&termAttr);
    //baudRate = B115200;          /* Not needed */
    cfsetispeed(&termAttr,B2000000);
    cfsetospeed(&termAttr,B2000000);
    termAttr.c_cflag &= ~PARENB;
    termAttr.c_cflag &= ~CSTOPB;
    termAttr.c_cflag &= ~CSIZE;
    termAttr.c_cflag |= CS8;
    termAttr.c_cflag |= (CLOCAL | CREAD);
    termAttr.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    termAttr.c_iflag &= ~(IXON | IXOFF | IXANY);
    termAttr.c_oflag &= ~OPOST;
    tcsetattr(fd,TCSANOW,&termAttr);
    
    // printf("UART1 configured....\n");
    usleep(10000);
    connected = 1;

    write_lock.lock();
    stm_reset_sub_ = n.subscribe("stm_comm/reset", 1, stm_reset);
    write_lock.unlock();

    ros::spinOnce();
    int debug = getenv("DEBUG")?atoi(getenv("DEBUG")):0;
    // double wheel_speed = getenv("WSP")?atoi(getenv("WSP")):10;
    double hz=50;
    ros::Rate rate(hz);
    double dt_ = 1.0/hz;
    // OmniDriver* div;
    // div = new OmniDriver(&n);
    ROS_INFO("waiting for stm sync");
    while(start_omni==false){
        
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("into the main code");
    // ros::Time prev = ros::Time::now();
    while(ros::ok()){ 
        double vmx= vx;
        double vmy= vy;
        double wmp = wp ; // Body frame
        
        double v1, v2, v3;
        v1 = - (L * wmp - (vmx / 2) - (sqrt3by2 * vmy));
        v2 = - (vmx + L * wmp);
        v3 = - (L * wmp - (vmx / 2) + (sqrt3by2 * vmy));

        //v1=v2=v3=1;
        // wheel_.lpos = wheel_.lpos + div->l_w_dir*wheel_speed*v1*dt_/R;
        // wheel_.bpos = wheel_.bpos + div->b_w_dir*wheel_speed*v2*dt_/R;
        // wheel_.rpos = wheel_.rpos + div->r_w_dir*wheel_speed*v3*dt_/R;
        std_msgs::Float64 wheel_vel;
        wheel_vel.data = v1;
        lf_wheel_callback(wheel_vel);
        wheel_vel.data = v2;
        bk_wheel_callback(wheel_vel);
        wheel_vel.data = v3;
        rt_wheel_callback(wheel_vel);

        //publishOdom();
        // ros::Duration d = ros::Time::now() - prev;
        // //ROS_INFO("loop rate %f",1./d.toSec());
        // prev = ros::Time::now();
        ros::spinOnce();
        rate.sleep();
        
    }
    printf("exiting stm-connect");
    close(fd);
    exit(0);
    return 0;
}

void signal_handler_IO (int status)
{
    // printf("received data from UART.\n");
    uint8_t x;
    int result;
    
    result = read(fd, buf, (int)BUF_SIZE);
    buf[result]= 0;
    inp_parse(result);
    //printf("%s\n",buf);
    
}

void inp_parse(int res){

    int result=1;
    /// add your data here to the msg
    buf[res];
    std::string inter=""; 
    int i=0;
    while(buf[i]=='|')i++;
    //printf("%s",buf);
    for(;i<res;i++){
        //printf("%c",buf[i]);
	    if(buf[i]==0)break;
        if(buf[i]=='|'){
            //std::cout<<inter<<"\n";
            if(inter!=""){
                int n = inter.length();
                if(n<=3) break;
                int st = inter.find(":");
                if (st==-1) break;
                try{
                    switch(inter[st-3]){
                        case 'l':
                            if(inter[st-1]=='v')
                                jnt_st.velocity[0] = std::stod(inter.substr(st+1));
                            else if(inter[st-1]=='p')
                                jnt_st.position[0] = std::stod(inter.substr(st+1));
                            break;
                        case 'r':
                            if(inter[st-1]=='v')
                                jnt_st.velocity[2] = std::stod(inter.substr(st+1));
                            else if(inter[st-1]=='p')
                                jnt_st.position[2] = std::stod(inter.substr(st+1));
                            break;
                        case 'b':
                            if(inter[st-1]=='v')
                                jnt_st.velocity[1] = std::stod(inter.substr(st+1));
                            else if(inter[st-1]=='p')
                                jnt_st.position[1] = std::stod(inter.substr(st+1));
                            break;
                    }
                } catch(std::invalid_argument& e){
                    ROS_WARN("Failed to parse STM input %s", inter);
                }
            } else {
                break;
            }
            inter = "";
        } else {
            inter.push_back(buf[i]);
        }
    }
    //std::cout<<inter<<"\n";
    if(result==-1) // Validate the string
        return;

    jnt_state_callback(jnt_st);
}

void lf_wheel_callback(const std_msgs::Float64& msg){
    double value = msg.data;
    //std::cout<<value<<"\n";
    char msg_data[16];
    sprintf(msg_data, "l_v:%.3f|\r\n",value);
    int n = strlen(msg_data);
    //printf("%s, %d",msg_data,n);
    write_lock.lock();
    write(fd, msg_data, n);
    write_lock.unlock();
    //usleep(8*100);
}

void rt_wheel_callback(const std_msgs::Float64& msg){
    double value = msg.data;
    char msg_data[16];
    sprintf(msg_data, "r_v:%.3f|\r\n",value);
    int n = strlen(msg_data);
    //printf("%s, %d",msg_data,n);
    write_lock.lock();
    write(fd, msg_data, n);
    write_lock.unlock();
    //usleep(8*100);
}

void bk_wheel_callback(const std_msgs::Float64& msg){
    double value = msg.data;
    char msg_data[16];
    sprintf(msg_data, "b_v:%.3f|\r\n",value);
    int n = strlen(msg_data);
    //printf("%s, %d",msg_data,n);
    write_lock.lock();
    write(fd, msg_data, n);
    write_lock.unlock();
    //usleep(8*100);
}

void stm_reset(const std_msgs::Empty& msg){
    write_lock.lock();
    write(fd, "|\n", 2);
    usleep(8*25);
    tcflush(fd, TCIOFLUSH);
    write_lock.unlock();
}
