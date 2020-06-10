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
