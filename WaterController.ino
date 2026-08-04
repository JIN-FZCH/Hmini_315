#include "ControlState.h"

/* Water-only closed-loop controller and four-thruster mixer. */

float errSum[4] = {0, 0, 0, 0};
float water_yaw_target = 0.0f;
int water_last_surge = 1500;
bool water_controller_initialized = false;

void resetWaterControllerState() {
  for (uint8_t i = 0; i < 4; ++i) {
    errSum[i] = 0;
  }
  water_yaw_target = psi;
  water_last_surge = 1500;
  water_controller_initialized = false;
}

void Marine_Remote_315() {
  if (!water_controller_initialized) {
    water_yaw_target = psi;
    water_last_surge = pitch_315;
    water_controller_initialized = true;
  }

  if (abs(water_last_surge - 1500) <= 3 &&
      abs(water_last_surge - pitch_315) > 3) {
    water_yaw_target = psi;
  }
  water_last_surge = pitch_315;

  float e_yaw = water_yaw_target - psi;
  if (e_yaw > 180) {
    e_yaw -= 360;
  }
  if (e_yaw <= -180) {
    e_yaw += 360;
  }

  const float Thr = pitch_315 - 1500;
  const float Yaw = yaw_315 - 1500;

  if (!depth_control_enabled) {
    errSum[0] = 0;
    errSum[2] = 0;
  }

  const float e_depth =
    depth_control_enabled ? depth_d_315 - Depth : 0.0f;
  const float e_roll = -phi;
  const float e_pitch = depth_control_enabled ? -the : 0.0f;
  float error[4] = {e_depth, e_roll, e_pitch, e_yaw};
  float *u = PID_outloop(error);

  float u_depth = constrain(u[0], -400, 400);
  float u_roll = u[1];
  float u_pitch = constrain(u[2], -150, 150);
  float u_yaw = constrain(u[3], -100, 100);

  /* The current four-thruster layout has no roll actuator. */
  (void)u_roll;

  int pwm1 = 1490;
  int pwm2 = 1490;
  if (depth_control_enabled) {
    pwm1 = constrain(1490 + u_depth + u_pitch, 1100, 1900);
    pwm2 = constrain(1490 + u_depth - u_pitch, 1100, 1900);
  }
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
