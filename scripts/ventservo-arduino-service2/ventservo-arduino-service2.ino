/* ventservo-arduino-service
 * Copyright Industrial Xaptation Limited 2020
 * All Rights Reserved.
 *
 * Emergency ventilator motor-contol sketch. Drives a Closed-loop Hybrid-Stepper motor to compress an Ambu-bag or similar.
 *
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
/* Arduino Nano pins
*/
const int pulsePos = 3;
const int directionPos = 5;
const int enableofflinePos = 6;

/* NodeMCU Pins
const int pulsePos = 4;
const int directionPos = 5;
const int enableofflinePos = 2;
*/

ventservo::servoruntime vsStatus;

// Actual RT parameters.
bool rt_motorState = false;
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
int crt_spr = 1600;
float crt_sa = 55.0;
float crt_ir = 3.0;
float crt_er = 6.0;
float crt_ih = 300.0;
float crt_eh = 500.0;
int crt_steps = 267;


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


void setup(){
  //Serial.begin(57600);
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
  /*
  rt_motorState = 0;
  rt_spr = 1600;
  rt_sa = 60.0;
  rt_ir = 3.0;
  rt_ih = 250.0;
  rt_er = 6.0;
  rt_eh = 350.0;
  rt_steps = 267;
  // pub_interval < 15 not recommended
  */
  steps = 0;
  pub_interval = 50;
  rt_cc = 0;
}

void loop(){
  pub_vsStatus();
  motorStateCtl();
  activate_crt();
  while (rt_motorState){
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
