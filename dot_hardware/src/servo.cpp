#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <cstdlib>
#include <unistd.h>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <math.h>
#include <pigpio.h>


#define NUM_GPIO 32

#define MIN_WIDTH 1000
#define MAX_WIDTH 2000

int run = 1;

void cmd_callback(const geometry_msgs::Point& msg);

void coor2ang(float x, float y, float* ang){
   x = std::min(std::max(x, (float)-1.5708), (float)1.5708);
   y = std::min(std::max(y, (float)-1.5708), (float)1.5708);
   ang[0] = sin(x);
   ang[1] = sin(y);
}

int randint(int from, int to)
{
   return (random() % (to - from + 1)) + from;
}

void stop(int signum)
{
   run = 0;
}

class ServoControlRos{
   public:
      geometry_msgs::Point p;
      int cnxs[2];
      int axss[2];
      float offs[2];
      float d1, d2;
      float val1, val2;
      float m;
      bool new_goal;
      float tar[2];

      ServoControlRos(bool pi_shift=false):d1(0.001),d2(0.001),m(0.5),new_goal(false) 
      {
         // GPIO
         cnxs[0] = getenv("SERVO_X")?atoi(getenv("SERVO_X")):23;
         cnxs[1] = getenv("SERVO_Y")?atoi(getenv("SERVO_Y")):24;
         axss[0] = getenv("SER_X_DIR")?atoi(getenv("SER_X_DIR")):2;
         axss[1] = getenv("SER_Y_DIR")?atoi(getenv("SER_Y_DIR")):2;
	 offs[0] = getenv("SER_X_OFF")?(float)atoi(getenv("SER_X_OFF"))/100:0;
	 offs[1] = getenv("SER_Y_OFF")?(float)atoi(getenv("SER_Y_OFF"))/100:0;
         ROS_INFO("offsets %f, %f", offs[0], offs[1]); 
	if(pi_shift)
            val1 = -m;
         else
            val1 = 0;
         val2 = 0;
         tar[0] = val1;
         tar[1] = val2;
      }
      void setTarget(float x, float y){
         coor2ang(x, y, tar);
      }

      int servoMap(float inp){
         int cmd = (((inp+1)/2)*((int)MAX_WIDTH-(int)MIN_WIDTH)) + (int)MIN_WIDTH;
         return std::min(std::max((int)MIN_WIDTH, cmd), (int)MAX_WIDTH);
      }

      void serControl(int cnx, float val){
         gpioServo(cnx, servoMap(val));
      }

      void control(){
         val1 = 0.8*val1 + 0.2*tar[0];
         val2 = 0.8*val2 + 0.2*tar[1];
         serControl(cnxs[0], (axss[0]-1)*(val1+offs[0]) );
         serControl(cnxs[1], (axss[1]-1)*(val2+offs[1]) );
      }
      bool is_reached(){
         if((abs(tar[0]-val1) + abs(tar[1]-val2))<0.01)
			   return true;
   		else
			   return false;
      }
      void stop(){
         gpioServo(cnxs[0], 0);
         gpioServo(cnxs[1], 0);
      }
};

ServoControlRos ser;
ros::Subscriber ser_cmd_;

void cmd_callback(const geometry_msgs::Point& msg){
   ser.p.x = msg.x;
   ser.p.y = msg.y;
   double mag = sqrt(pow(ser.p.x,2) + pow(ser.p.y, 2));
   //ROS_INFO("%f",(float)mag);
   if (mag<0.1){
      ser.new_goal = false;
      ser.p.x = 0;
      ser.p.y = 0;
   }else{
      ser.new_goal = true;
   }
   if(mag>2.808){
      ser.p.x /= mag;
      ser.p.y /= mag;
   }
}


int main(int argc, char *argv[])
{
   //int i, g;
   
   if (gpioInitialise() < 0) return -1;
   gpioSetSignalFunc(SIGINT, stop);
   ros::init(argc, argv, "servo_node");
   ros::NodeHandle n;
   double rate = 20;
   ser_cmd_ = n.subscribe("servo_cmd", 10, cmd_callback);

   while(ros::ok()){
      if(ser.new_goal){
         ser.setTarget(ser.p.x, ser.p.y);
         ser.control();
         if(ser.is_reached()){
            ser.new_goal = false;
         }
      } else {
         ser.setTarget(0,0);
         ser.control();
      }
      ros::spinOnce();
      time_sleep(1./rate);
      
      //ROS_INFO("GREAT DEBUG %d",ser.new_goal);
   }
   ser.stop();
 
   gpioTerminate();

   return 0;
}
