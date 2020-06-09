/*
 * ventservo-arduino-service
 */

#include <ros.h>
#include <ros/time.h>
#include <Chrono.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <ventservo/servoruntime.h>
#include <ventservo/servort.h>

ros::NodeHandle  nh;

using ventservo::servort;

// pins for Motor Control:
const int pulsePos = 3;
const int directionPos = 5;
const int enableofflinePos = 6;

ventservo::servoruntime vsStatus;

// Actual RT parameters.
bool rt_motorState;
int rt_spr;
float rt_sa;
float rt_ir;
float rt_er;
float rt_ih;
float rt_eh;
int rt_steps;
int steps;
int pub_interval;
int rt_cc;

// Configured RT paramters.
int crt_spr;
float crt_sa;
float crt_ir;
float crt_er;
float crt_ih;
float crt_eh;
int crt_steps;


ros::Publisher ventservoStatus("ventservo_status", &vsStatus);

Chrono driveIns;
Chrono driveExs;
Chrono holdIns;
Chrono holdExs;
Chrono statusPub;
Chrono pulseSpeed;

void server_config(const servort::Request & req, servort::Response & res){
  rt_motorState = req.servo_state;
  crt_spr = req.steps_per_revolution;
  crt_sa = req.servo_angle;
  crt_ir = req.inspiratory_rate;
  crt_er = req.expiratory_rate;
  crt_ih = req.inspiratory_hold;
  crt_eh = req.expiratory_hold;
  pub_interval = req.publish_interval;
  
  res.servo_state = rt_motorState;
  res.steps_per_revolution = crt_spr;
  res.servo_angle = crt_sa;
  res.inspiratory_rate = crt_ir;
  res.expiratory_rate = crt_er;
  res.inspiratory_hold = crt_ih;
  res.expiratory_hold = crt_eh;
  res.publish_interval = pub_interval;
  res.currentTime = nh.now();
}

ros::ServiceServer<servort::Request, servort::Response> srv_config("ventservo_rtsrv_config",&server_config);

int calc_steps(int rt_spr, float rt_sa){
  float degrees_per_step = float(rt_spr) / float(360);
  float float_steps = degrees_per_step * rt_sa;
  float_steps = round(float_steps);
  int steps = int(float_steps);
  if(int(steps)){
    return steps;
  }
  else{
    return 0;
  }
}

void pub_vsStatus(){
  if(statusPub.hasPassed(pub_interval)){
    statusPub.restart();
    vsStatus.servo_state = rt_motorState;
    vsStatus.steps_per_revolution = rt_spr;
    vsStatus.servo_angle = rt_sa;
    vsStatus.inspiratory_rate = rt_ir;
    vsStatus.expiratory_rate = rt_er;
    vsStatus.inspiratory_hold = rt_ih;
    vsStatus.expiratory_hold = rt_eh;
    vsStatus.publish_interval = pub_interval;
    vsStatus.motor_position_steps = steps;
    vsStatus.cycles_complete = rt_cc;
    vsStatus.currentTime = nh.now();
    
    ventservoStatus.publish( &vsStatus );
  }
}
void drive_inspiration(){
  pub_vsStatus();
  digitalWrite(directionPos, HIGH);  
  driveIns.restart();  
  while(steps < rt_steps){
    pub_vsStatus();
    if(driveIns.hasPassed(rt_ir)){
      driveIns.restart();
      pub_vsStatus();
      digitalWrite(pulsePos, HIGH);
      digitalWrite(pulsePos, LOW);        
      steps++;
    }
  }
}

void drive_expiration(){
  pub_vsStatus();
  digitalWrite(directionPos, LOW);
  driveExs.restart();
  while(steps > 0){
    pub_vsStatus();
    if(driveExs.hasPassed(rt_er)){
      driveExs.restart();
      pub_vsStatus();
      digitalWrite(pulsePos, HIGH);
      digitalWrite(pulsePos, LOW);        
      steps--;      
    }
  }
}

void inspiration_hold(){
  pub_vsStatus();
  holdIns.restart();
  while(true){
    pub_vsStatus();
    if(holdIns.hasPassed(rt_ih)){
      pub_vsStatus();
      break;
    }
  }
}


void expiration_hold(){
  pub_vsStatus();
  holdExs.restart();
  while(true){
    pub_vsStatus();
    if(holdExs.hasPassed(rt_eh)){
      pub_vsStatus();
      break;
    }
  }
}

void motorStateCtl(){
  if(rt_motorState){
    digitalWrite(enableofflinePos, LOW);
  }
  else {
    digitalWrite(enableofflinePos, HIGH);
  }
}

void activate_crt(){
  // Make configured the actual RT parameters.
  rt_spr = crt_spr;
  rt_sa = crt_sa;
  rt_ir = crt_ir;
  rt_ih = crt_ih;
  rt_er = crt_er;
  rt_eh = crt_eh;
}

void setup(){
  // make the pins outputs:
  pinMode(pulsePos, OUTPUT);
  pinMode(directionPos, OUTPUT);
  pinMode(enableofflinePos, OUTPUT);

  digitalWrite(pulsePos, LOW);
  digitalWrite(directionPos, LOW);
  digitalWrite(enableofflinePos, HIGH);
  
  nh.initNode();
  nh.advertiseService(srv_config);
  nh.advertise(ventservoStatus);

// Default motor settings.
  rt_motorState = 0;     
  rt_spr = 1600;
  rt_sa = 60.0;
  rt_ir = 2.0;
  rt_ih = 200.0;
  rt_er = 3.0;
  rt_eh = 300.0;
  rt_steps = 267;
  steps = 0;
  // pub_interval < 15 not recommended
  pub_interval = 50;
  rt_cc = 0;
}

void loop(){
  pub_vsStatus();
  motorStateCtl();
  while (rt_motorState){
    activate_crt();
    rt_steps = calc_steps(rt_spr,rt_sa);
    drive_inspiration();
    inspiration_hold();
    drive_expiration();
    expiration_hold();
    rt_cc++;
    motorStateCtl();
    nh.spinOnce();    
  }
  nh.spinOnce();    
}
