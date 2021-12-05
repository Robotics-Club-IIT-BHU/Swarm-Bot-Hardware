/*
* file: stm_comm_node.cpp
* author: Somnath Sendhil Kumar
* data: 5th Dec 2021
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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

#define BUF_SIZE 2097152
// 2MB

ros::Publisher jnt_state_pub_;
ros::Subscriber l_wheel_cmd_;
ros::Subscriber r_wheel_cmd_;
ros::Subscriber b_wheel_cmd_;
sensor_msgs::JointState jnt_st;

void signal_handler_IO (int status);   /* definition of signal handler */
void inp_parse(int res);
void lf_wheel_callback(const std_msgs::Float64::ConstPtr& msg);
void rt_wheel_callback(const std_msgs::Float64::ConstPtr& msg);
void bk_wheel_callback(const std_msgs::Float64::ConstPtr& msg);

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
    l_wheel_cmd_ = n.subscribe("left_joint_velocity_controller/command", 1000, lf_wheel_callback);
    r_wheel_cmd_ = n.subscribe("right_joint_velocity_controller/command", 1000, rt_wheel_callback);
    b_wheel_cmd_ = n.subscribe("back_joint_velocity_controller/command", 1000, bk_wheel_callback);

    jnt_st.header.frame_id = "base_link";
    jnt_st.header.stamp = ros::Time::now();
    jnt_st.name = std::vector<std::string>(3,0);
    jnt_st.name[0] = "left_joint";
    jnt_st.name[1] = "right_joint";
    jnt_st.name[2] = "back_joint";

    jnt_st.position = std::vector<double> (3,0);
    jnt_st.velocity = std::vector<double> (3,0);
    jnt_st.effort = std::vector<double> (3,0);

    fd = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1)
    {
       perror("open_port: Unable to open /dev/ttyO1\n");
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
    printf("UART1 configured....\n");

    connected = 1;
    while(connected == 1){
          usleep(2500000);
    }

    close(fd);
    exit(0);
}

void signal_handler_IO (int status)
{
    printf("received data from UART.\n");
    uint8_t x;
    int result;
    
    result = read(fd, buf, (int)BUF_SIZE);
    buf[result]= 0;
    inp_parse(result);
    // printf("%s\n",buf);
    
}

void inp_parse(int res){

    int result=1;
    /// add your data here to the msg
    buf[res];
    std::string inter=""; 

    for(int i=0;i<res;i++){
        if(buf[i]=='|'){
            if(inter!=""){
                switch(inter[0]){
                    case 'l':
                        if(inter[2]=='v')
                            jnt_st.velocity[0] = std::stod(inter.substr(4));
                        else if(inter[2]=='p')
                            jnt_st.position[0] = std::stod(inter.substr(4));
                        break;
                    case 'r':
                        if(inter[2]=='v')
                            jnt_st.velocity[1] = std::stod(inter.substr(4));
                        else if(inter[2]=='p')
                            jnt_st.position[1] = std::stod(inter.substr(4));
                        break;
                    case 'b':
                        if(inter[2]=='v')
                            jnt_st.velocity[2] = std::stod(inter.substr(4));
                        else if(inter[2]=='p')
                            jnt_st.position[2] = std::stod(inter.substr(4));
                        break;
                }
            } else {
                break;
            }
        } else {
            inter += buf[i];
        }
    }
    if(result==-1) // Validate the string
        return;

    jnt_state_pub_.publish(jnt_st);
}

void lf_wheel_callback(std_msgs::Float64::ConstPtr& msg){
    double value = msg->data;
    char msg_data[8];
    sprintf(msg_data, "lf:%.2f|",value);
    write(fd, msg_data, 8);
    usleep(8*100);
}

void rt_wheel_callback(std_msgs::Float64::ConstPtr& msg){
    double value = msg->data;
    char msg_data[8];
    sprintf(msg_data, "rt:%.2f|",value);
    write(fd, msg_data, 8);
    usleep(8*100);
}

void bk_wheel_callback(std_msgs::Float64::ConstPtr& msg){
    double value = msg->data;
    char msg_data[8];
    sprintf(msg_data, "bk:%.2f|",value);
    write(fd, msg_data, 8);
    usleep(8*100);
}