#include <ros.h>
#include <std_msgs/String.h>
#include <SPI.h>
#include <std_msgs/Empty.h>
#include "motor.h"
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

HardwareSerial ros_pot(PA10, PA9);

Motor* l;
Motor* r;
Motor* b;

long pos=0;
long inter_val=0;

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
    if((millis() - l->prev_check)>5){ // update at 200 hz
      l->curr_vel = (double)MILLIINV*(l->read()-l->read(l->prev_pos))/(double)(millis() - l->prev_check);
      l->prev_check = millis();
      l->prev_pos = l->pos;
    }
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
    if((millis() - b->prev_check)>5){ // update at 200 hz
      b->curr_vel = (double)MILLIINV*(b->read()-b->read(l->prev_pos))/(double)(millis() - b->prev_check);
      b->prev_check = millis();
      b->prev_pos = b->pos;
    }
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
    if((millis() - r->prev_check)>5){ // update at 200 hz
      r->curr_vel = (double)MILLIINV*(r->read()-r->read(l->prev_pos))/(double)(millis() - r->prev_check);
      r->prev_check = millis();
      r->prev_pos = r->pos;
    }
}


class ModHardware: public ArduinoHardware
{
  public:
    ModHardware():ArduinoHardware(&ros_pot, 230400){}; //115200){};
};

void messageCb(const std_msgs::Empty& toggle_msg){
  digitalWrite(PC13, HIGH-digitalRead(PC13));
}



ros::NodeHandle_<ModHardware> nh;

void bJointVel(const std_msgs::Float64& msg){
  b->setVel(msg.data);
}

void rJointVel(const std_msgs::Float64& msg){
  r->setVel(msg.data);
}

void lJointVel(const std_msgs::Float64& msg){
  l->setVel(msg.data);
}

std_msgs::String str_msg;
sensor_msgs::JointState jnt_st;

// ros::Publisher chatter("chatter", &str_msg);
ros::Publisher joint_state_pub("joint_states", &jnt_st);

// char hello[13] = "hello world!";
// ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb);
ros::Subscriber<std_msgs::Float64> l_joint_sub("velocity_controller/left_joint_vel_controller/command", &lJointVel);
ros::Subscriber<std_msgs::Float64> r_joint_sub("velocity_controller/right_joint_vel_controller/command", &rJointVel);
ros::Subscriber<std_msgs::Float64> b_joint_sub("velocity_controller/back_joint_vel_controller/command", &bJointVel);


void setup() {
  ros_pot.begin(115200);
  pinMode(PC13, OUTPUT);
  nh.initNode();
  // nh.advertise(chatter);
  nh.advertise(joint_state_pub);
  nh.subscribe(l_joint_sub);
  nh.subscribe(b_joint_sub);
  nh.subscribe(r_joint_sub);
  // nh.subscribe(sub);
  jnt_st.header.frame_id = "base_link";
  jnt_st.name = (char** )malloc(sizeof(char[32])*3);
  jnt_st.name_length = 3;
  jnt_st.name[0] = "left_wheel_joint";
  jnt_st.name[1] = "right_wheel_joint";
  jnt_st.name[2] = "back_wheel_joint";
  jnt_st.position = (float *)malloc(sizeof(float)*3);
  jnt_st.position_length = 3;
  jnt_st.velocity = (float *)malloc(sizeof(float)*3);
  jnt_st.velocity_length = 3;
  jnt_st.effort = (float *)malloc(sizeof(float)*3);
  jnt_st.effort_length = 3;

  for(int i=0; i<3;i++)
    jnt_st.position[i] = 0;
    
  for(int i=0; i<3;i++)
    jnt_st.velocity[i] = 0;
  
  for(int i=0; i<3;i++)
    jnt_st.effort[i] = 0;
  
  l = new Motor(LMOTOR_P, LMOTOR_N, LMOTOR_M, LMOTOR_ENCA, LMOTOR_ENCB, 100, 0, 0);
  r = new Motor(RMOTOR_P, RMOTOR_N, RMOTOR_M, RMOTOR_ENCA, RMOTOR_ENCB, 100, 0, 0);
  b = new Motor(BMOTOR_P, BMOTOR_N, BMOTOR_M, BMOTOR_ENCA, BMOTOR_ENCB, 100, 0, 0);
  
  pinMode(l->motor_encA, INPUT);
  pinMode(l->motor_encB, INPUT);
  pinMode(l->motor_p, OUTPUT);
  pinMode(l->motor_n, OUTPUT);
  pinMode(l->motor_e, OUTPUT);
  pinMode(b->motor_encA, INPUT);
  pinMode(b->motor_encB, INPUT);
  pinMode(b->motor_p, OUTPUT);
  pinMode(b->motor_n, OUTPUT);
  pinMode(b->motor_e, OUTPUT);
  pinMode(r->motor_encA, INPUT);
  pinMode(r->motor_encB, INPUT);
  pinMode(r->motor_p, OUTPUT);
  pinMode(r->motor_n, OUTPUT);
  pinMode(r->motor_e, OUTPUT);
  digitalWrite(l->motor_p, HIGH);
  digitalWrite(l->motor_n, LOW);
  analogWrite(l->motor_e, 63350);
  attachInterrupt(digitalPinToInterrupt(l->motor_encA), updateEncoderL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(l->motor_encB), updateEncoderL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(b->motor_encA), updateEncoderB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(b->motor_encB), updateEncoderB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(r->motor_encA), updateEncoderR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(r->motor_encB), updateEncoderR, CHANGE);
  
}

void loop() {
  // str_msg.data = hello;
  // chatter.publish(&str_msg);
  jnt_st.header.stamp = nh.now();
  jnt_st.position[0] = l->read();
  jnt_st.position[1] = r->read();
  jnt_st.position[2] = b->read();

  if((millis() - l->prev_check)>5){ // update at 200 hz
    l->curr_vel = (double)MILLIINV*(l->read()-l->read(l->prev_pos))/(double)(millis() - l->prev_check);
    l->prev_check = millis();
    l->prev_pos = l->pos;
  }
  
  if((millis() - b->prev_check)>5){ // update at 200 hz
    b->curr_vel = (double)MILLIINV*(b->read()-b->read(b->prev_pos))/(double)(millis() - b->prev_check);
    b->prev_check = millis();
    b->prev_pos = b->pos;
  }
  
  if((millis() - r->prev_check)>5){ // update at 200 hz
    r->curr_vel = (double)MILLIINV*(r->read()-r->read(r->prev_pos))/(double)(millis() - r->prev_check);
    r->prev_check = millis();
    r->prev_pos = r->pos;
  }
  
  l->control();
  r->control();
  b->control();
  jnt_st.velocity[0] = l->curr_vel;
  jnt_st.velocity[1] = r->curr_vel;
  jnt_st.velocity[2] = b->curr_vel;

  joint_state_pub.publish(&jnt_st);

  


  nh.spinOnce();
//  ros_pot.print("print");
  //delay(10); // 100 hz
}
