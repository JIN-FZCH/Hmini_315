#include "ControlState.h"

/* Hardware output layer. Permission decisions come from ControlManager. */

const uint8_t ESC_FRONT_DEPTH_PIN = 7;
const uint8_t ESC_REAR_DEPTH_PIN = 10;
const uint8_t ESC_LEFT_YAW_PIN = 9;
const uint8_t ESC_RIGHT_YAW_PIN = 8;
const uint8_t SERVO_LEFT_PIN = 4;
const uint8_t SERVO_RIGHT_PIN = 6;

const int THRUSTER_NEUTRAL_US = 1490;
const int SERVO_SAFE_LEFT_US = 1000;
const int SERVO_MIRROR_SUM_US = 3000;
const uint16_t SERVO_SLEW_US_PER_SECOND = 500U;
const uint16_t SERVO_UPDATE_PERIOD_MS = 20U;
const uint16_t SERVO_MAX_ELAPSED_MS = 100U;
const uint32_t SERVO_RX_LOSS_HOLD_MS = 2000UL;
const uint16_t WATER_CONTROL_PERIOD_MS = 50U;

Servo esc1;
Servo esc2;
Servo esc3;
Servo esc4;
Servo servo_left;
Servo servo_right;

int servo_output_us = SERVO_SAFE_LEFT_US;
int servo_motion_target_us = SERVO_SAFE_LEFT_US;
bool servo_motion_active = false;
uint32_t servo_last_update_ms = 0;
uint32_t water_control_last_update_ms = 0;

void writeNeutralThrusterOutputs() {
  esc1.writeMicroseconds(THRUSTER_NEUTRAL_US);
  esc2.writeMicroseconds(THRUSTER_NEUTRAL_US);
  esc3.writeMicroseconds(THRUSTER_NEUTRAL_US);
  esc4.writeMicroseconds(THRUSTER_NEUTRAL_US);
}

void writeSafeServoOutputs() {
  servo_output_us = SERVO_SAFE_LEFT_US;
  servo_motion_target_us = SERVO_SAFE_LEFT_US;
  servo_motion_active = false;

  servo_left.writeMicroseconds(servo_output_us);
  servo_right.writeMicroseconds(
    SERVO_MIRROR_SUM_US - servo_output_us
  );
}

void writeSafeActuatorOutputs() {
  writeNeutralThrusterOutputs();
  writeSafeServoOutputs();
}

void actuatorOutputsBegin() {
  /* Preload safe pulse values before attach() to avoid an initial jump. */
  writeSafeActuatorOutputs();
  esc1.attach(ESC_FRONT_DEPTH_PIN);
  esc2.attach(ESC_REAR_DEPTH_PIN);
  esc3.attach(ESC_LEFT_YAW_PIN);
  esc4.attach(ESC_RIGHT_YAW_PIN);
  servo_left.attach(SERVO_LEFT_PIN, 544, 2400);
  servo_right.attach(SERVO_RIGHT_PIN, 544, 2400);
  writeSafeActuatorOutputs();

  servo_last_update_ms = millis();
  water_control_last_update_ms = servo_last_update_ms;
  Serial.println(F("EVENT,actuators=DISARMED,reason=STARTUP"));
}

bool actuatorOutputsArmed() {
  return waterControlCommandAllowed();
}

void updateServoOutputs(uint32_t now) {
  if (!servoControlCommandAllowed()) {
    const bool hold_during_air_rx_loss =
      system_control_state == SYSTEM_STATE_RX_FAILSAFE &&
      control_mode == CONTROL_MODE_AIR &&
      rx315LastFrameAgeMs() <= SERVO_RX_LOSS_HOLD_MS;

    if (hold_during_air_rx_loss) {
      /* Keep the last pulse pair; Servo continues generating those pulses. */
      servo_last_update_ms = now;
      return;
    }

    if (servo_output_us != SERVO_SAFE_LEFT_US ||
        servo_motion_target_us != SERVO_SAFE_LEFT_US ||
        servo_motion_active) {
      writeSafeServoOutputs();
    }
    /* Do not accumulate disabled time into the next slew-rate step. */
    servo_last_update_ms = now;
    return;
  }

  const uint32_t elapsed = now - servo_last_update_ms;
  if (elapsed < SERVO_UPDATE_PERIOD_MS) {
    return;
  }
  servo_last_update_ms = now;

  /* Continuously follow the knob while limiting the physical servo speed. */
  servo_motion_target_us = constrain(angle, 1000, 2000);
  servo_motion_active = servo_output_us != servo_motion_target_us;
  if (!servo_motion_active) {
    return;
  }

  const uint32_t limited_elapsed =
    elapsed > SERVO_MAX_ELAPSED_MS
      ? SERVO_MAX_ELAPSED_MS
      : elapsed;
  int max_step = (int)(
    ((uint32_t)SERVO_SLEW_US_PER_SECOND * limited_elapsed) / 1000UL
  );
  if (max_step < 1) {
    max_step = 1;
  }

  if (servo_output_us < servo_motion_target_us) {
    servo_output_us += min(
      max_step,
      servo_motion_target_us - servo_output_us
    );
  } else if (servo_output_us > servo_motion_target_us) {
    servo_output_us -= min(
      max_step,
      servo_output_us - servo_motion_target_us
    );
  }

  servo_motion_active = servo_output_us != servo_motion_target_us;

  servo_left.writeMicroseconds(servo_output_us);
  servo_right.writeMicroseconds(
    SERVO_MIRROR_SUM_US - servo_output_us
  );
}

void updateActuatorOutputs(uint32_t now) {
  updateServoOutputs(now);

  if (!waterControlCommandAllowed()) {
    writeNeutralThrusterOutputs();
    return;
  }

  if (now - water_control_last_update_ms < WATER_CONTROL_PERIOD_MS) {
    return;
  }
  water_control_last_update_ms = now;
  Marine_Remote_315();
}
