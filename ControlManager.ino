#include "ControlState.h"

/*
 * Central control policy.
 *
 * This is the only module that decides the final system state. Output modules
 * consume that state and do not rebuild mode, link and sensor conditions.
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
SystemControlState system_control_state = SYSTEM_STATE_RX_FAILSAFE;
bool depth_control_enabled = false;

int yaw_315 = CONTROL_STICK_MID;
int pitch_315 = CONTROL_STICK_MID;
float depth_d_315 = 0.0f;
int angle = 1000;

bool control_manager_initialized = false;
bool control_system_state_initialized = false;
bool fresh_stop_observed = false;
bool water_mode_entry_allowed = false;
bool water_rearm_ready = false;
const char *control_system_state_reason = "STARTUP";

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

const char *systemControlStateName(SystemControlState state) {
  switch (state) {
    case SYSTEM_STATE_STOP_SAFE:
      return "STOP_SAFE";
    case SYSTEM_STATE_AIR_ACTIVE:
      return "AIR_ACTIVE";
    case SYSTEM_STATE_WATER_LOCKED:
      return "WATER_LOCKED";
    case SYSTEM_STATE_WATER_ACTIVE:
      return "WATER_ACTIVE";
    default:
      return "RX_FAILSAFE";
  }
}

const char *systemControlStateReason() {
  return control_system_state_reason;
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

bool controlWaterEntrySafe(bool fresh,
                           uint16_t yaw_pwm,
                           uint16_t surge_pwm,
                           uint16_t depth_pwm) {
  return fresh &&
         abs((int)yaw_pwm - (int)CONTROL_STICK_MID) <=
           (int)CONTROL_ENTRY_DEADBAND &&
         abs((int)surge_pwm - (int)CONTROL_STICK_MID) <=
           (int)CONTROL_ENTRY_DEADBAND &&
         depth_pwm <= CONTROL_ENTRY_DEPTH_MAX;
}

const char *controlWaterEntryGuardReason(uint16_t yaw_pwm,
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
  return "ENTRY_SAFE";
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

void controlSetSystemState(SystemControlState next_state,
                           const char *reason) {
  const bool state_changed =
    !control_system_state_initialized ||
    next_state != system_control_state;
  const bool reason_changed =
    strcmp(reason, control_system_state_reason) != 0;

  if (!state_changed && !reason_changed) {
    return;
  }

  const bool was_water_active =
    control_system_state_initialized &&
    system_control_state == SYSTEM_STATE_WATER_ACTIVE;
  const bool water_active = next_state == SYSTEM_STATE_WATER_ACTIVE;

  system_control_state = next_state;
  control_system_state_reason = reason;
  control_system_state_initialized = true;

  if (state_changed) {
    resetWaterControllerState();
  }

  Serial.print(F("EVENT,state="));
  Serial.print(systemControlStateName(system_control_state));
  Serial.print(F(",reason="));
  Serial.println(reason);

  if (was_water_active != water_active) {
    Serial.print(F("EVENT,actuators="));
    Serial.print(water_active ? F("ARMED") : F("DISARMED"));
    Serial.print(F(",reason="));
    Serial.println(reason);
  }
}

void updateControlManager(uint32_t now) {
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
  angle = constrain((int)servo_pwm, 1000, 2000);
  depth_control_enabled = false;

  const ControlMode requested_mode =
    controlDecodeMode(fresh, mode_pwm);
  const bool mode_changed =
    !control_manager_initialized || requested_mode != control_mode;

  if (mode_changed) {
    const bool had_previous_mode = control_manager_initialized;
    const ControlMode previous_mode = control_mode;
    control_mode = requested_mode;
    control_manager_initialized = true;
    water_mode_entry_allowed =
      control_mode == CONTROL_MODE_WATER &&
      had_previous_mode &&
      previous_mode == CONTROL_MODE_STOP &&
      fresh_stop_observed;
    if (control_mode == CONTROL_MODE_WATER) {
      fresh_stop_observed = false;
    }
    water_rearm_ready = water_mode_entry_allowed;
    resetWaterControllerState();
    controlPrintMode(fresh, mode_pwm);
  }

  if (!fresh) {
    fresh_stop_observed = false;
    water_rearm_ready = false;
    controlSetSystemState(SYSTEM_STATE_RX_FAILSAFE, "RX_LOST");
    return;
  }

  if (control_mode == CONTROL_MODE_AIR) {
    fresh_stop_observed = false;
    water_mode_entry_allowed = false;
    water_rearm_ready = false;
    controlSetSystemState(SYSTEM_STATE_AIR_ACTIVE, "AIR_MODE");
    return;
  }

  if (control_mode == CONTROL_MODE_STOP) {
    fresh_stop_observed = true;
    water_mode_entry_allowed = false;
    water_rearm_ready = false;
    controlSetSystemState(SYSTEM_STATE_STOP_SAFE, "STOP_MODE");
    return;
  }

  const bool entry_safe = controlWaterEntrySafe(
    fresh, yaw_pwm, surge_pwm, depth_pwm
  );

  if (!water_mode_entry_allowed) {
    controlSetSystemState(
      SYSTEM_STATE_WATER_LOCKED,
      "STOP_REQUIRED"
    );
    return;
  }

  if (system_control_state == SYSTEM_STATE_WATER_ACTIVE) {
    if (!imuDataFresh(now)) {
      water_rearm_ready = false;
      controlSetSystemState(SYSTEM_STATE_WATER_LOCKED, "IMU_TIMEOUT");
      return;
    }
    if (!depthDataFresh(now)) {
      water_rearm_ready = false;
      controlSetSystemState(SYSTEM_STATE_WATER_LOCKED, "DEPTH_TIMEOUT");
      return;
    }

    depth_control_enabled = depth_pwm > CONTROL_DEPTH_OFF_MAX;
    return;
  }

  if (!entry_safe) {
    water_rearm_ready = true;
    controlSetSystemState(
      SYSTEM_STATE_WATER_LOCKED,
      controlWaterEntryGuardReason(yaw_pwm, surge_pwm, depth_pwm)
    );
    return;
  }

  if (!water_rearm_ready) {
    controlSetSystemState(
      SYSTEM_STATE_WATER_LOCKED,
      "REENTRY_REQUIRED"
    );
    return;
  }

  if (!imuDataFresh(now)) {
    water_rearm_ready = false;
    controlSetSystemState(SYSTEM_STATE_WATER_LOCKED, "IMU_STALE");
    return;
  }

  if (!depthDataFresh(now)) {
    water_rearm_ready = false;
    controlSetSystemState(SYSTEM_STATE_WATER_LOCKED, "DEPTH_STALE");
    return;
  }

  water_rearm_ready = false;
  controlSetSystemState(SYSTEM_STATE_WATER_ACTIVE, "ENTRY_SAFE");
  depth_control_enabled = depth_pwm > CONTROL_DEPTH_OFF_MAX;
}

bool controlModeAllowsSbus() {
  return system_control_state == SYSTEM_STATE_AIR_ACTIVE;
}

bool waterControlCommandAllowed() {
  return system_control_state == SYSTEM_STATE_WATER_ACTIVE;
}

bool servoControlCommandAllowed() {
  return system_control_state == SYSTEM_STATE_AIR_ACTIVE;
}
