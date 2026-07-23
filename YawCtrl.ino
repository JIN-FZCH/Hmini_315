void TD(float *x, float u) {
  float r = 100;
  float h = 0.01;
  x[0] = x[0] + h * x[1];
  x[1] = x[1] + h * fst(x, u, r, h);
}
float fst(float *x, float u, float r, float h) {
  float y = 0;
  float a0 = 0;
  float a = 0;
  float fhan = 0;
  float d = 0;
  float d0 = 0;

  d = r * h;
  d0 = h * d;
  y = x[0] - u + h * (x[1]);

  a0 = sqrt(d * d + 8 * r * abs(y));

  if (abs(y) > d0) {
    a = x[1] + 0.5 * (a0 - d) * sgn(y);
  }
  else {
    a = x[1] + y / h;
  }
  if (abs(a) > d) {
    fhan = -r * sgn(a);
  }
  else {
    fhan = -r * a / d;
  }
  return (fhan);
}
float sgn(float x) {
  float u = (x > 0) - (x < 0);
  return u;
}

float *PID_outloop(float error[]) {
  float kp[4] = {1000, 0.2, 15, 5};
  float ki[4] = {5, 0, 0, 0};
  float kd[4] = {1, 0.1, 2, 2};
  static float output[4] = {0, 0, 0, 0}; //必须是static
  float dErr[4] = {0, 0, 0, 0};
  static float depth_[2] = {Depth, 0};
  TD(depth_, Depth);
  float d_depth = depth_[1];
  dErr[0] = - d_depth;
  dErr[1] = - d_att[0];
  dErr[2] = - d_att[1];
  dErr[3] = - d_att[2];
  static unsigned long lastTime = millis() - 50;
  unsigned long now = millis();
//  double timeChange = (double)(now - lastTime) / 1000.0;
  double timeChange = 0.05;
  for (int i = 0; i < 4; i++) {
    errSum[i] = errSum[i] + (error[i] * timeChange);
    errSum[i] = constrain(errSum[i], -50/ki[i], 50/ki[i]);
    output[i] = kp[i] * error[i] + ki[i] * errSum[i] + kd[i] * dErr[i];
  }
  lastTime = now;
  return output;
}

float PID_Remote(float psi_d, float psi) {
  float kp = 3;
  float ki = 0.2;
  float kd = 1;
  static unsigned long lastTime = millis() - 50;
  unsigned long now = millis();
//  double timeChange = (double)(now - lastTime) / 1000;
  double timeChange = 0.05;
  float error = psi_d - psi;
  errSum_yaw += (error * timeChange);
  errSum_yaw = constrain(errSum_yaw, -50 / ki, 50 / ki);
  float dErr = -d_att[2];;
  float output = kp * error + ki * errSum_yaw + kd * dErr;
  lastTime = now;
  return output;
}
