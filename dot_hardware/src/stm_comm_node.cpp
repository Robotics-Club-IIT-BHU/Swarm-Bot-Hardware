/*
* file: stm_comm_node.cpp
* author: Somnath Sendhil Kumar
* data: 5th Dec 2021
*/
#include <stdio.h>
#include <iostream>
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

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

#define BUF_SIZE 256
// 2MB

ros::Publisher jnt_state_pub_;
ros::Subscriber l_wheel_cmd_;
ros::Subscriber r_wheel_cmd_;
ros::Subscriber b_wheel_cmd_;
sensor_msgs::JointState jnt_st;

void signal_handler_IO (int status);   /* definition of signal handler */
void inp_parse(int res);
void lf_wheel_callback(const std_msgs::Float64& msg);
void rt_wheel_callback(const std_msgs::Float64& msg);
void bk_wheel_callback(const std_msgs::Float64& msg);

int n;
int fd;
int connected;
struct termios termAttr;
struct sigaction saio;
char buf[BUF_SIZE];


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "stm_comm");
    ros::NodeHandle n;
    jnt_state_pub_ = n.advertise<sensor_msgs::JointState>("joint_state",100);
    l_wheel_cmd_ = n.subscribe("velocity_controller/left_joint_vel_controller/command", 1000, lf_wheel_callback);
    r_wheel_cmd_ = n.subscribe("velocity_controller/right_joint_vel_controller/command", 1000, rt_wheel_callback);
    b_wheel_cmd_ = n.subscribe("velocity_controller/back_joint_vel_controller/command", 1000, bk_wheel_callback);

    jnt_st.header.frame_id = "base_link";
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

    connected = 1;
    //while(connected == 1){
          
    //      usleep(2500000);
    //}
    ros::spin();
    close(fd);
    exit(0);
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

    jnt_state_pub_.publish(jnt_st);
}

void lf_wheel_callback(const std_msgs::Float64& msg){
    double value = msg.data;
    //std::cout<<value<<"\n";
    char msg_data[16];
    sprintf(msg_data, "l_v:%.3f|",value);
    int n = strlen(msg_data);
    printf("%s, %d",msg_data,n);
    write(fd, msg_data, n);
    //usleep(8*100);
}

void rt_wheel_callback(const std_msgs::Float64& msg){
    double value = msg.data;
    char msg_data[16];
    sprintf(msg_data, "r_v:%.3f|",value);
    int n = strlen(msg_data);
    printf("%s, %d",msg_data,n);
    write(fd, msg_data, n);
    //usleep(8*100);
}

void bk_wheel_callback(const std_msgs::Float64& msg){
    double value = msg.data;
    char msg_data[16];
    sprintf(msg_data, "b_v:%.3f|",value);
    int n = strlen(msg_data);
    printf("%s, %d",msg_data,n);
    write(fd, msg_data, n);
    //usleep(8*100);
}
