#include "imu_data_decode.h"
#include "packet.h"
#include "ControlState.h"
#include <Wire.h>
#include "MS5837.h"
#include <SPI.h>
#include <SD.h>
#include <Servo.h>

/*
 * 完整外设模式。ESC和舵机在setup()中初始化。STOP/AIR模式始终
 * 禁用本地执行器；从STOP切入WATER且入口与传感器检查通过后，
 * 本地执行器自动启用。
 */
#define RX315_BENCH_TEST_ONLY 0

const uint8_t ESC_FRONT_DEPTH_PIN = 7;
const uint8_t ESC_REAR_DEPTH_PIN = 10;
const uint8_t ESC_LEFT_YAW_PIN = 9;
const uint8_t ESC_RIGHT_YAW_PIN = 8;
const uint8_t SERVO_LEFT_PIN = 4;
const uint8_t SERVO_RIGHT_PIN = 6;
const float DEPTH_ZERO_DEADBAND_M = 0.03f;
const int SERVO_SAFE_LEFT_US = 1000;
const int SERVO_MIRROR_SUM_US = 3000;
const uint16_t SERVO_SLEW_US_PER_SECOND = 500U;
const uint16_t SERVO_UPDATE_PERIOD_MS = 20U;
const uint16_t SERVO_MAX_ELAPSED_MS = 100U;
const uint16_t SERVO_ENDPOINT_DWELL_MS = 500U;

MS5837 sensor;

String SDdata = "";
File dataFile;
String FileName = "12101.txt";

Servo esc1;
Servo esc2;
Servo esc3;
Servo esc4;
Servo servo_left;
Servo servo_right;

float Depth = 0;
float Depth0 = 0;
float Temp = 0;

int yaw_315 = 1500;
int pitch_315 = 1500;
float depth_d_315 = 0;
int angle = 1000;
int servo_output_us = SERVO_SAFE_LEFT_US;
int servo_motion_target_us = SERVO_SAFE_LEFT_US;
bool servo_motion_active = false;
uint32_t servo_last_update_ms = 0;
uint32_t servo_motion_complete_ms = 0;

float phi = 0;
float the = 0;
float psi = 0;
float d_att[4] = {0, 0, 0, 0};

float errSum[4] = {0, 0, 0, 0};
unsigned long time_now = 0;

const uint32_t IMU_FRESH_TIMEOUT_MS = 250UL;
const uint32_t DEPTH_FRESH_TIMEOUT_MS = 250UL;
const uint8_t IMU_REQUIRED_BITMAP = BIT_VALID_EUL | BIT_VALID_GYR;

bool depth_sensor_initialized = false;
bool depth_data_valid = false;
uint32_t depth_last_update_ms = 0;
uint32_t imu_last_update_ms = 0;
uint32_t imu_last_frame_count = 0;
bool actuator_outputs_armed = false;

void SDwrite() {
  if (dataFile) {
    dataFile.println(SDdata);
  }
  SDdata = "";
}

void writeNeutralThrusterOutputs() {
  esc1.writeMicroseconds(1490);
  esc2.writeMicroseconds(1490);
  esc3.writeMicroseconds(1490);
  esc4.writeMicroseconds(1490);
}

void writeSafeServoOutputs() {
  servo_output_us = SERVO_SAFE_LEFT_US;
  servo_motion_target_us = SERVO_SAFE_LEFT_US;
  servo_motion_active = false;
  servo_motion_complete_ms = millis();

  servo_left.writeMicroseconds(servo_output_us);
  servo_right.writeMicroseconds(
    SERVO_MIRROR_SUM_US - servo_output_us
  );
}

void writeNeutralOutputs() {
  writeNeutralThrusterOutputs();
  writeSafeServoOutputs();
}

void updateServoOutputs(uint32_t now) {
  const bool servo_allowed =
    rx315HasFreshFrame() &&
    control_mode == CONTROL_MODE_AIR;

  if (!servo_allowed) {
    writeSafeServoOutputs();
    return;
  }

  const uint32_t elapsed = now - servo_last_update_ms;
  if (elapsed < SERVO_UPDATE_PERIOD_MS) {
    return;
  }
  servo_last_update_ms = now;

  /*
   * A command is latched only while idle. Changes arriving during motion are
   * deliberately ignored; after reaching the endpoint and dwelling there,
   * only the knob's latest state may start the next complete motion.
   */
  if (!servo_motion_active) {
    if (now - servo_motion_complete_ms < SERVO_ENDPOINT_DWELL_MS) {
      return;
    }

    const int requested_target = constrain(angle, 1000, 2000);
    if (requested_target == servo_output_us) {
      return;
    }
    servo_motion_target_us = requested_target;
    servo_motion_active = true;
  }

  const uint32_t limited_elapsed =
    elapsed > SERVO_MAX_ELAPSED_MS
      ? SERVO_MAX_ELAPSED_MS
      : elapsed;
  int max_step = (int)(
    ((uint32_t)SERVO_SLEW_US_PER_SECOND * limited_elapsed) /
    1000UL
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

  if (servo_output_us == servo_motion_target_us) {
    servo_motion_active = false;
    servo_motion_complete_ms = now;
  }

  servo_left.writeMicroseconds(servo_output_us);
  servo_right.writeMicroseconds(
    SERVO_MIRROR_SUM_US - servo_output_us
  );
}

bool imuDataFresh(uint32_t now) {
  return imu_last_frame_count > 0 &&
         (bitmap & IMU_REQUIRED_BITMAP) == IMU_REQUIRED_BITMAP &&
         now - imu_last_update_ms <= IMU_FRESH_TIMEOUT_MS;
}

bool depthDataFresh(uint32_t now) {
  return depth_sensor_initialized &&
         depth_data_valid &&
         now - depth_last_update_ms <= DEPTH_FRESH_TIMEOUT_MS;
}

bool actuatorOutputsArmed() {
  return actuator_outputs_armed;
}

void armActuatorOutputs(const char *reason) {
  actuator_outputs_armed = true;
  resetWaterControllerState();
  Serial.print(F("EVENT,actuators=ARMED,reason="));
  Serial.println(reason);
}

void disarmActuatorOutputs(const char *reason) {
  const bool was_armed = actuator_outputs_armed;
  actuator_outputs_armed = false;
  writeNeutralOutputs();
  resetWaterControllerState();

  if (was_armed) {
    Serial.print(F("EVENT,actuators=DISARMED,reason="));
    Serial.println(reason);
  }
}

void enforceActuatorOutputSafety(uint32_t now) {
  if (!actuator_outputs_armed) {
    return;
  }

  if (!rx315HasFreshFrame()) {
    disarmActuatorOutputs("RX_LOST");
  } else if (control_mode != CONTROL_MODE_WATER) {
    disarmActuatorOutputs("MODE_NOT_WATER");
  } else if (!water_control_unlocked) {
    disarmActuatorOutputs("WATER_LOCKED");
  } else if (!imuDataFresh(now)) {
    disarmActuatorOutputs("IMU_TIMEOUT");
  } else if (!depthDataFresh(now)) {
    disarmActuatorOutputs("DEPTH_TIMEOUT");
  }
}

void setup() {
  Serial.begin(115200);
  rx315Begin();
  sbusOutputBegin();
  Serial.println(F("CONSOLE_READY,type=help"));

#if RX315_BENCH_TEST_ONLY
  return;
#endif

  pinMode(A10, OUTPUT);
  pinMode(A11, OUTPUT);
  digitalWrite(A10, LOW);
  digitalWrite(A11, LOW);

  /*
   * Servo objects default to 1500 us. Preload every target before attach()
   * so the first generated pulse is already the defined safe output.
   */
  writeNeutralOutputs();
  esc1.attach(ESC_FRONT_DEPTH_PIN);
  esc2.attach(ESC_REAR_DEPTH_PIN);
  esc3.attach(ESC_LEFT_YAW_PIN);
  esc4.attach(ESC_RIGHT_YAW_PIN);
  servo_left.attach(SERVO_LEFT_PIN, 544, 2400);
  servo_right.attach(SERVO_RIGHT_PIN, 544, 2400);
  writeNeutralOutputs();
  servo_last_update_ms = millis();
  servo_motion_complete_ms = servo_last_update_ms;
  Serial.println(F("EVENT,actuators=DISARMED,reason=STARTUP"));

  Serial1.begin(115200);
  imu_data_decode_init();

  pinMode(53, OUTPUT);
  if (!SD.begin(53)) {
    Serial.println(F("SD initialization failed!"));
    Serial.println(F("SD logging disabled; control and SBUS remain available"));
  } else {
    dataFile = SD.open(FileName, FILE_WRITE);
    if (!dataFile) {
      Serial.println(F("SD file open failed; logging disabled"));
    }
  }

  Wire.begin();
  depth_sensor_initialized = sensor.init();
  if (!depth_sensor_initialized) {
    Serial.println(F("Init failed!"));
    Serial.println(F("Are SDA/SCL connected correctly?"));
    Serial.println(F("Blue Robotics Bar30: White=SDA, Green=SCL"));
    Serial.println(F("Water control inhibited: depth sensor unavailable"));
  } else {
    sensor.setModel(MS5837::MS5837_30BA);
    sensor.setFluidDensity(1000);
    sensor.read();
    Depth0 = sensor.depth();
    Depth = 0;
    Temp = sensor.temperature();
    depth_data_valid = true;
    depth_last_update_ms = millis();
  }

}

void loop() {
  rx315Update();
  updateControlState();
  sbusOutputUpdate();
  serialConsoleUpdate();

#if RX315_BENCH_TEST_ONLY
  return;
#endif

  const uint32_t now = millis();
  if (frame_count != imu_last_frame_count) {
    imu_last_frame_count = frame_count;
    imu_last_update_ms = now;
  }

  if (imu_last_frame_count > 0) {
    phi = id0x91.eul[1];
    the = -id0x91.eul[0];
    psi = id0x91.eul[2];
    d_att[0] = id0x91.gyr[1];
    d_att[1] = -id0x91.gyr[0];
    d_att[2] = id0x91.gyr[2];
  }

  if (depth_sensor_initialized) {
    sensor.read_nodelay();
    if (sensor.check_step() == 3) {
      const float new_depth = sensor.depth() - Depth0;
      const float new_temp = sensor.temperature();
      if (isfinite(new_depth) && isfinite(new_temp)) {
        Depth =
          fabs(new_depth) <= DEPTH_ZERO_DEADBAND_M ? 0.0f : new_depth;
        Temp = new_temp;
        depth_data_valid = true;
        depth_last_update_ms = now;
      } else {
        depth_data_valid = false;
      }
    }
  }

  enforceActuatorOutputSafety(now);
  updateServoOutputs(now);

  if (now - time_now >= 50) {
    time_now = now;

    const bool imu_fresh = imuDataFresh(now);
    const bool depth_fresh = depthDataFresh(now);

    if (actuator_outputs_armed &&
        waterControlCommandAllowed() &&
        imu_fresh &&
        depth_fresh) {
      Marine_Remote_315();
    } else {
      writeNeutralThrusterOutputs();
      resetWaterControllerState();

      if (control_mode == CONTROL_MODE_WATER &&
          water_control_unlocked) {
        if (!imu_fresh) {
          invalidateWaterControl("IMU_TIMEOUT");
        } else if (!depth_fresh) {
          invalidateWaterControl("DEPTH_TIMEOUT");
        }
      }
    }
  }
}

void serialEvent1() {
  while (Serial1.available()) {
    packet_decode((uint8_t)Serial1.read());
  }
}
