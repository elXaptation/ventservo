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

