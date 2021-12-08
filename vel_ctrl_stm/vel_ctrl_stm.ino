#include "motor.h"

HardwareSerial ros_pot(PA10, PA9);

Motor* l;
Motor* r;
Motor* b;

long pos=0;
long inter_val=0;
long unsigned int prev_time;
int cnt;

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
    // if((millis() - l->prev_check)>5){ // update at 200 hz
    //   l->curr_vel = (double)MILLIINV*(l->read()-l->read(l->prev_pos))/(double)(millis() - l->prev_check);
    //   l->prev_check = millis();
    //   l->prev_pos = l->pos;
    // }
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
    // if((millis() - b->prev_check)>5){ // update at 200 hz
    //   b->curr_vel = (double)MILLIINV*(b->read()-b->read(l->prev_pos))/(double)(millis() - b->prev_check);
    //   b->prev_check = millis();
    //   b->prev_pos = b->pos;
    // }
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
    // if((millis() - r->prev_check)>5){ // update at 200 hz
    //   r->curr_vel = (double)MILLIINV*(r->read()-r->read(l->prev_pos))/(double)(millis() - r->prev_check);
    //   r->prev_check = millis();
    //   r->prev_pos = r->pos;
    // }
}

void readCMD(){
  bool start = true;
  bool flashout = false;
  int cnt = 0;
  double buf_signi = 0;
  double denom = 1;
  bool dec = false;
  while(ros_pot.available()){
    char c = ros_pot.read();
    if(c=='|'){
      if(start)continue;
      start = true;
      dec = false;
      if(cnt==0){
        // ros_pot.println(buf_signi/denom,3);
        l->setVel(buf_signi/denom);
      }else if (cnt==1){
        // ros_pot.println(buf_signi/denom,3);
        b->setVel(buf_signi/denom);
      }else if (cnt==2) {
        // ros_pot.println(buf_signi/denom,3);
        r->setVel(buf_signi/denom);
      }
      buf_signi = 0;
      denom = 1;
    }else{
      if(start){
        start = false;
        if(c=='l'){
          cnt = 0;
        } else if(c=='b'){
          cnt = 1;
        } else if(c=='r'){
          cnt = 2;
        } else {
          cnt = -1;
        }
      } else {
        if(c=='.'){
          dec = true;
        } else {
          if(c>'9'||c<'0')continue;
          buf_signi *=10;
          buf_signi += (int)(c-'0');  
          if(dec){
            denom *= 10;
          }
        }
      }
    }
  }
}

void writeState(double l_pos, double b_pos, double r_pos, double l_vel, double b_vel, double r_vel){
  ros_pot.print("|l_p:");
  ros_pot.print(l_pos,3);
  ros_pot.print("|b_p:");
  ros_pot.print(b_pos,3);
  ros_pot.print("|r_p:");
  ros_pot.print(r_pos,3);
  ros_pot.print("|l_v:");
  ros_pot.print(l_vel,3);
  ros_pot.print("|b_v:");
  ros_pot.print(b_vel,3);
  ros_pot.print("|r_v:");
  ros_pot.print(r_vel,3);
  ros_pot.print("|\n");
}
void setup() {
  ros_pot.begin(2000000);
  pinMode(PC13, OUTPUT);
  
  l = new Motor(LMOTOR_P, LMOTOR_N, LMOTOR_M, LMOTOR_ENCA, LMOTOR_ENCB, 1, 0, 0);
  r = new Motor(RMOTOR_P, RMOTOR_N, RMOTOR_M, RMOTOR_ENCA, RMOTOR_ENCB, 1, 0, 0);
  b = new Motor(BMOTOR_P, BMOTOR_N, BMOTOR_M, BMOTOR_ENCA, BMOTOR_ENCB, 1, 0, 0);
  
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
  // digitalWrite(l->motor_p, HIGH);
  // digitalWrite(l->motor_n, LOW);
  // analogWrite(l->motor_e, 63350);
  attachInterrupt(digitalPinToInterrupt(l->motor_encA), updateEncoderL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(l->motor_encB), updateEncoderL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(b->motor_encA), updateEncoderB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(b->motor_encB), updateEncoderB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(r->motor_encA), updateEncoderR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(r->motor_encB), updateEncoderR, CHANGE);
  prev_time = micros();
  cnt = 0;
}

void loop() {
  prev_time = micros();
  readCMD();
  
  if((millis() - l->prev_check)>20){ // update at 200 hz
    l->curr_vel = (double)MILLIINV*(l->read()-l->read(l->prev_pos))/(double)(millis() - l->prev_check);
    l->prev_check = millis();
    l->prev_pos = l->pos;
  }
  
  if((millis() - b->prev_check)>20){ // update at 200 hz
    b->curr_vel = (double)MILLIINV*(b->read()-b->read(b->prev_pos))/(double)(millis() - b->prev_check);
    b->prev_check = millis();
    b->prev_pos = b->pos;
  }
  
  if((millis() - r->prev_check)>20){ // update at 200 hz
    r->curr_vel = (double)MILLIINV*(r->read()-r->read(r->prev_pos))/(double)(millis() - r->prev_check);
    r->prev_check = millis();
    r->prev_pos = r->pos;
  }
  
  l->control();
  r->control();
  b->control();

  cnt++;
  if (cnt == 10){ // Rate/10
    writeState(l->read(), b->read(), r->read(), l->curr_vel, b->curr_vel, r->curr_vel);
    cnt = 0;
  }
  //delay(10); // 100 hz
  long unsigned int dt = (micros() - prev_time);
  delayMicroseconds(max(((long unsigned int)666 - dt), (long unsigned int)0)); // 1.5KHz  = 1000000/1500 = 666.66
  //ros_pot.print("********************************");
  //ros_pot.print(1000000./(micros()-prev_time));
  //ros_pot.print("********************************\n");
  //prev_time = micros();
}
