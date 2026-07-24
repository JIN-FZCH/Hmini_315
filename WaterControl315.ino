/*
 * 315 MHz remote water-thruster control.
 *
 * Channel-to-control mapping is intentionally not performed here yet.
 * roll_315, pitch_315 and depth_d_315 must only be updated after each
 * physical handset control has been identified.
 */

void Marine_Remote_315() {
  static float psi_d = psi;
  static float pitch_last = pitch_315;

  if (abs(pitch_last - 1500) <= 3 &&
      abs(pitch_last - pitch_315) > 3) {
    psi_d = psi;
  }
  pitch_last = pitch_315;

  float e_yaw = psi_d - psi;
  if (e_yaw > 180) {
    e_yaw -= 360;
  }
  if (e_yaw <= -180) {
    e_yaw += 360;
  }

  const float Thr = pitch_315 - 1500;
  const float Yaw = roll_315 - 1500;

  if (depth_d_315 == 0) {
    errSum[0] = 0;
    errSum[1] = 0;
    errSum[2] = 0;
    errSum[3] = 0;
  }

  const float e_depth = depth_d_315 - Depth;
  const float e_roll = -phi;
  const float e_pitch = -the;
  float error[4] = {e_depth, e_roll, e_pitch, e_yaw};
  float *u = PID_outloop(error);

  float u_depth = constrain(u[0], -400, 400);
  float u_pitch = constrain(u[2], -150, 150);
  float u_yaw = constrain(u[3], -100, 100);

  int pwm1 = constrain(1490 + u_depth + u_pitch, 1000, 2000);
  int pwm2 = constrain(1490 + u_depth - u_pitch, 1000, 2000);
  esc1.writeMicroseconds(pwm1);
  esc2.writeMicroseconds(pwm2);

  float thr1;
  float thr2;
  if (abs(pitch_315 - 1500) <= 10) {
    thr1 = 1490 + Thr + Yaw;
    thr2 = 1490 + Thr - Yaw;
  } else {
    thr1 = 1490 + Thr - u_yaw + Yaw;
    thr2 = 1490 + Thr + u_yaw - Yaw;
  }
  thr1 = constrain(thr1, 1000, 2000);
  thr2 = constrain(thr2, 1000, 2000);
  esc3.writeMicroseconds(thr1);
  esc4.writeMicroseconds(thr2);

  SDdata.concat(phi);
  SDdata.concat(',');
  SDdata.concat(the);
  SDdata.concat(',');
  SDdata.concat(psi);
  SDdata.concat(',');
  SDdata.concat(depth_d_315);
  SDdata.concat(',');
  SDdata.concat(Depth);
  SDdata.concat(',');
  SDdata.concat(u_depth);
  SDdata.concat(',');
  SDdata.concat(u_pitch);
  SDdata.concat(',');
  SDdata.concat(u_yaw);
  SDdata.concat(',');
  SDdata.concat(pwm1);
  SDdata.concat(',');
  SDdata.concat(pwm2);
  SDdata.concat(',');
  SDdata.concat(thr1);
  SDdata.concat(',');
  SDdata.concat(thr2);
  SDwrite();
}
