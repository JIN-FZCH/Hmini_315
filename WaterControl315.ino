#include "ControlState.h"

/*
 * 315 MHz handset control and local water-actuator control.
 *
 * CH1 is yaw. CH4 is currently unassigned by the control layer, although its
 * raw value remains available and is forwarded in the SBUS CH4 slot. CH2
 * keeps the legacy pitch_315 name, but its local water-mode meaning is surge
 * (forward/reverse thrust), not pitch angle.
 */

const uint8_t CONTROL_CH_YAW = 0;
const uint8_t CONTROL_CH_SURGE = 1;
const uint8_t CONTROL_CH_DEPTH = 2;
const uint8_t CONTROL_CH_MODE = 6;
const uint8_t CONTROL_CH_SERVO = 9;

const uint16_t CONTROL_MODE_AIR_MAX = 1250U;
const uint16_t CONTROL_MODE_WATER_MIN = 1750U;
const uint16_t CONTROL_STICK_MID = 1500U;
const uint16_t CONTROL_STICK_DEADBAND = 10U;
const uint16_t CONTROL_ENTRY_DEADBAND = 30U;
const uint16_t CONTROL_ENTRY_DEPTH_MAX = 1050U;
const uint16_t CONTROL_DEPTH_OFF_MAX = 1050U;

ControlMode control_mode = CONTROL_MODE_STOP;
bool water_entry_safe = false;
bool water_control_unlocked = false;
bool depth_control_enabled = false;

float water_yaw_target = 0;
int water_last_surge = 1500;
bool water_controller_initialized = false;

bool control_mode_initialized = false;
bool control_last_entry_safe = false;
bool control_water_lock_reported = false;

const char *controlModeName(ControlMode mode) {
  switch (mode) {
    case CONTROL_MODE_AIR:
      return "AIR";
    case CONTROL_MODE_WATER:
      return "WATER";
    default:
      return "STOP";
  }
}

int controlApplyStickDeadband(uint16_t pwm) {
  const int value = constrain((int)pwm, 1000, 2000);
  if (abs(value - (int)CONTROL_STICK_MID) <=
      (int)CONTROL_STICK_DEADBAND) {
    return CONTROL_STICK_MID;
  }
  return value;
}

float controlMapDepth(uint16_t pwm) {
  const int value = constrain((int)pwm, 1000, 2000);
  /*
   * The bottom region explicitly disables both vertical thrusters. Above
   * that region, map the remaining travel to a 0...1 m depth setpoint.
   */
  if (value <= (int)CONTROL_DEPTH_OFF_MAX) {
    return 0.0f;
  }
  return (value - (int)CONTROL_DEPTH_OFF_MAX) /
         (2000.0f - (float)CONTROL_DEPTH_OFF_MAX);
}

ControlMode controlDecodeMode(bool fresh, uint16_t mode_pwm) {
  if (!fresh) {
    return CONTROL_MODE_STOP;
  }
  if (mode_pwm <= CONTROL_MODE_AIR_MAX) {
    return CONTROL_MODE_AIR;
  }
  if (mode_pwm >= CONTROL_MODE_WATER_MIN) {
    return CONTROL_MODE_WATER;
  }
  return CONTROL_MODE_STOP;
}

const char *controlEntryGuardReason(uint16_t yaw_pwm,
                                    uint16_t surge_pwm,
                                    uint16_t depth_pwm) {
  if (abs((int)yaw_pwm - (int)CONTROL_STICK_MID) >
      (int)CONTROL_ENTRY_DEADBAND) {
    return "YAW_NOT_CENTERED";
  }
  if (abs((int)surge_pwm - (int)CONTROL_STICK_MID) >
      (int)CONTROL_ENTRY_DEADBAND) {
    return "SURGE_NOT_CENTERED";
  }
  if (depth_pwm > CONTROL_ENTRY_DEPTH_MAX) {
    return "DEPTH_NOT_ZERO";
  }
  return "READY";
}

void controlPrintMode(bool fresh, uint16_t mode_pwm) {
  Serial.print(F("EVENT,mode="));
  Serial.print(controlModeName(control_mode));
  Serial.print(F(",ch7="));
  if (fresh) {
    Serial.print(mode_pwm);
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(F(",rx="));
  Serial.println(fresh ? F("CONNECTED") : F("DISCONNECTED"));
}

void resetWaterControllerState() {
  for (uint8_t i = 0; i < 4; ++i) {
    errSum[i] = 0;
  }
  water_yaw_target = psi;
  water_last_surge = CONTROL_STICK_MID;
  water_controller_initialized = false;
}

void invalidateWaterControl(const char *reason) {
  if (!water_control_unlocked) {
    return;
  }

  water_control_unlocked = false;
  depth_control_enabled = false;
  disarmActuatorOutputs(reason);
  resetWaterControllerState();
  control_water_lock_reported = true;
  Serial.print(F("EVENT,water=LOCKED,reason="));
  Serial.println(reason);
}

void updateControlState() {
  const bool fresh = rx315HasFreshFrame();
  const uint16_t mode_pwm =
    fresh ? rx315Channel(CONTROL_CH_MODE) : CONTROL_STICK_MID;
  const uint16_t yaw_pwm =
    fresh ? rx315Channel(CONTROL_CH_YAW) : CONTROL_STICK_MID;
  const uint16_t surge_pwm =
    fresh ? rx315Channel(CONTROL_CH_SURGE) : CONTROL_STICK_MID;
  const uint16_t depth_pwm =
    fresh ? rx315Channel(CONTROL_CH_DEPTH) : 1000U;
  const uint16_t servo_pwm =
    fresh ? rx315Channel(CONTROL_CH_SERVO) : 1000U;

  yaw_315 = controlApplyStickDeadband(yaw_pwm);
  pitch_315 = controlApplyStickDeadband(surge_pwm);
  depth_d_315 = controlMapDepth(depth_pwm);
  depth_control_enabled =
    fresh && depth_pwm > CONTROL_DEPTH_OFF_MAX;
  angle = constrain((int)servo_pwm, 1000, 2000);

  const ControlMode requested_mode =
    controlDecodeMode(fresh, mode_pwm);
  const bool mode_changed =
    !control_mode_initialized || requested_mode != control_mode;

  if (mode_changed) {
    if (actuatorOutputsArmed()) {
      disarmActuatorOutputs(
        !fresh
          ? "RX_LOST"
          : requested_mode == CONTROL_MODE_STOP
              ? "STOP_MODE"
              : "MODE_CHANGE"
      );
    }
    water_control_unlocked = false;
    depth_control_enabled = false;
    control_mode = requested_mode;
    control_mode_initialized = true;
    water_entry_safe = false;
    control_last_entry_safe = false;
    control_water_lock_reported = false;
    resetWaterControllerState();
    controlPrintMode(fresh, mode_pwm);
  }

  if (control_mode != CONTROL_MODE_WATER) {
    water_entry_safe = false;
    water_control_unlocked = false;
    depth_control_enabled = false;
    return;
  }

  water_entry_safe =
    fresh &&
    abs((int)yaw_pwm - (int)CONTROL_STICK_MID) <=
      (int)CONTROL_ENTRY_DEADBAND &&
    abs((int)surge_pwm - (int)CONTROL_STICK_MID) <=
      (int)CONTROL_ENTRY_DEADBAND &&
    depth_pwm <= CONTROL_ENTRY_DEPTH_MAX;

  const bool entry_safe_rising =
    water_entry_safe && !control_last_entry_safe;

  if (!water_entry_safe && !control_water_lock_reported) {
    Serial.print(F("EVENT,water=LOCKED,reason="));
    Serial.println(
      fresh
        ? controlEntryGuardReason(yaw_pwm, surge_pwm, depth_pwm)
        : "RX_LOST"
    );
    control_water_lock_reported = true;
  }

  /*
   * Unlock only when the controls enter the safe region. If a sensor fault
   * revokes the unlock while the controls remain safe, the operator must
   * leave and re-enter the safe region (or cycle the mode switch).
   */
  if (entry_safe_rising && !water_control_unlocked) {
    control_water_lock_reported = true;
    const uint32_t now = millis();
    if (!imuDataFresh(now)) {
      Serial.println(F("EVENT,water=LOCKED,reason=IMU_STALE"));
    } else if (!depthDataFresh(now)) {
      Serial.println(F("EVENT,water=LOCKED,reason=DEPTH_STALE"));
    } else {
      water_control_unlocked = true;
      resetWaterControllerState();
      armActuatorOutputs("WATER_ENTRY_SAFE");
      Serial.println(F("EVENT,water=UNLOCKED,reason=ENTRY_SAFE"));
    }
  }
  control_last_entry_safe = water_entry_safe;
}

bool controlModeAllowsSbus() {
  return rx315HasFreshFrame() &&
         control_mode == CONTROL_MODE_AIR;
}

bool waterControlCommandAllowed() {
  return rx315HasFreshFrame() &&
         control_mode == CONTROL_MODE_WATER &&
         water_control_unlocked &&
         actuatorOutputsArmed();
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

  /*
   * u_roll is intentionally not mixed into an actuator. The current
   * four-thruster layout implements depth, pitch and yaw control only.
   */
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
