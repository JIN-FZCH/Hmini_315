void dataReceived() {
  servo_left.writeMicroseconds(servo_key);
  servo_right.writeMicroseconds(3000-servo_key);
  if (print1 > 1800) {
    Serial.print(Depth); Serial.print(", ");
    Serial.print(depth_d); Serial.print(", ");
    Serial.print(phi); Serial.print(", ");
    Serial.print(the); Serial.print(", ");
    Serial.print(psi); Serial.print(", ");
    Serial.print(sign); Serial.print(", ");
    Serial.print(throttle); Serial.println(", ");
  }
    if ((SD_key ==1)&&(SD1 > 1800)) {
      SDread();
      SD_key =0;
    }
    if (SD1 < 1800){
      SD_key =1;
    }
  if (((sign - 1800) > 0) && ((sign_last - 1800 <= 0))) {
    dataFile  = SD.open(FileName, FILE_WRITE);
    depth_d = Depth_d;
    psi_d0 = psi;
    sensor.read_nodelay();
    if (sensor.check_step() == 3) {
      Depth0 = sensor.depth();
    }
    Arduino_Start = millis();
  }
  if (((sign - 1800) < 0) && ((sign_last - 1800 >= 0))) {
      dataFile.close();
    }
  if (sign < 1200) {
    depth_d = 0;
    stop_flag = 0;
    errSum[0] = 0; errSum[1] = 0; errSum[2] = 0; errSum[3] = 0;
    errSum_yaw = 0;
    esc1.writeMicroseconds(1490);
    esc2.writeMicroseconds(1490);
    esc3.writeMicroseconds(1490);
    esc4.writeMicroseconds(1490);
  }
  else if (sign > 1800) {
    Marine_Remote();
  }
  else {
    Marine_Remote();
  }
  sign_last = sign;
}
void Marine_Remote() {
  static float psi_d = psi;
  static float pitch_last = pitch;
  if ((abs(pitch_last - 1500) <= 3) && (abs(pitch_last - pitch) > 3)) {
    psi_d = psi;
  }
  pitch_last = pitch;
  float e_yaw = (psi_d - psi);
  if (e_yaw > 180) {
    e_yaw = e_yaw - 360;
  }
  if (e_yaw <= -180) {
    e_yaw = e_yaw + 360;
  }
  float u_yaw = PID_Remote(psi_d, psi);
  u_yaw = constrain(u_yaw, -100, 100);
  float Thr = (pitch - 1500) * 1;
  float Yaw = (roll - 1500) * 1;
  float thr1 = 0;
  float thr2 = 0;

  if (abs(pitch - 1500) <= 10) {
    thr1 = 1490 + Thr + Yaw;
    thr2 = 1490 + Thr - Yaw;
  }
  else {
    thr1 = 1490 + Thr - u_yaw + Yaw;
    thr2 = 1490 + Thr + u_yaw - Yaw;
  }
  thr1 = constrain(thr1, 1000, 2000);
  thr2 = constrain(thr2, 1000, 2000);
  esc3.writeMicroseconds(thr1);
  esc4.writeMicroseconds(thr2);
  esc1.writeMicroseconds(1490);
  esc2.writeMicroseconds(1490);
}
void Marine_Remote_315() {
  static float psi_d = psi;
  static float pitch_last= pitch_315;
  if ((abs(pitch_last - 1500) <= 3) && (abs(pitch_last - pitch_315) > 3)) {
    psi_d = psi;
  }
  pitch_last = pitch_315;
  float e_yaw = (psi_d - psi);
  if (e_yaw > 180) {
    e_yaw = e_yaw - 360;
  }
  if (e_yaw <= -180) {
    e_yaw = e_yaw + 360;
  }
  float Thr = (pitch_315 - 1500) * 1;
  float Yaw = (roll_315 - 1500) * 1;

  if (depth_d_315 == 0) {
    errSum[0] = 0;errSum[1] = 0;errSum[2] = 0;errSum[3] = 0;
  }

  float e_depth = depth_d_315 - Depth;
  float phi_d = 0;
  float the_d = 0;
  float e_roll = (phi_d - phi);
  float e_pitch = (the_d - the);
  float error[4] = {e_depth, e_roll, e_pitch, e_yaw}; //深度,俯仰，偏航
  float *u = PID_outloop(error);
  float u_depth = *u;
  float u_roll = *(u + 1);
  float u_pitch = *(u + 2);
  float u_yaw = *(u + 3);
  u_depth = constrain(u_depth, -400, 400);
  u_pitch = constrain(u_pitch, -150, 150);
  u_yaw = constrain(u_yaw, -100, 100);
  int  pwm1 =1500;int  pwm2 =1500;
  pwm1 = 1490 + u_depth + u_pitch;
  pwm2 = 1490 + u_depth - u_pitch;
  pwm1 = constrain(pwm1, 1000, 2000);
  pwm2 = constrain(pwm2, 1000, 2000);
  esc1.writeMicroseconds(pwm1);
  esc2.writeMicroseconds(pwm2);
  
  float thr1 = 0;
  float thr2 = 0;
  if (abs(pitch_315 - 1500) <= 10) {
    thr1 = 1490 + Thr + Yaw;
    thr2 = 1490 + Thr - Yaw;
  }
  else {
    thr1 = 1490 + Thr - u_yaw + Yaw;
    thr2 = 1490 + Thr + u_yaw - Yaw;
  }
  thr1 = constrain(thr1, 1000, 2000);
  thr2 = constrain(thr2, 1000, 2000);
  esc3.writeMicroseconds(thr1);
  esc4.writeMicroseconds(thr2);   
  SDdata.concat(phi); SDdata.concat(',');
  SDdata.concat(the); SDdata.concat(',');
  SDdata.concat(psi); SDdata.concat(',');
  SDdata.concat(depth_d_315); SDdata.concat(',');
  SDdata.concat(Depth); SDdata.concat(',');
  SDdata.concat(u_depth); SDdata.concat(',');
  SDdata.concat(u_pitch); SDdata.concat(',');
  SDdata.concat(u_yaw); SDdata.concat(',');
  SDdata.concat(pwm1); SDdata.concat(',');
  SDdata.concat(pwm2); SDdata.concat(',');
  SDdata.concat(thr1); SDdata.concat(',');
  SDdata.concat(thr2); SDdata.concat(',');
  SDwrite();
}
