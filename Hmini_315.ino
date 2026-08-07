#include "imu_data_decode.h"
#include "packet.h"
#include "ControlState.h"
#include <Wire.h>
#include "MS5837.h"
#include <SPI.h>
#include <SD.h>

/*
 * Main sketch: initialize peripherals, refresh sensors, update the central
 * control manager, then let protocol and actuator modules consume its state.
 */
#define RX315_BENCH_TEST_ONLY 0

const char FIRMWARE_BUILD_ID[] = "HMINI315_20260806_01";

const float DEPTH_ZERO_DEADBAND_M = 0.03f;

MS5837 sensor;

String SDdata = "";
File dataFile;
String FileName = "12101.txt";

float Depth = 0;
float Depth0 = 0;
float Temp = 0;

float phi = 0;
float the = 0;
float psi = 0;
float d_att[4] = {0, 0, 0, 0};

const uint32_t IMU_FRESH_TIMEOUT_MS = 250UL;
const uint32_t DEPTH_FRESH_TIMEOUT_MS = 250UL;
const uint8_t IMU_REQUIRED_BITMAP = BIT_VALID_EUL | BIT_VALID_GYR;

bool depth_sensor_initialized = false;
bool depth_data_valid = false;
uint32_t depth_last_update_ms = 0;
uint32_t imu_last_update_ms = 0;
uint32_t imu_last_frame_count = 0;

void SDwrite() {
  if (dataFile) {
    dataFile.println(SDdata);
  }
  SDdata = "";
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

void setup() {
  Serial.begin(115200);
  Serial.print(F("FIRMWARE,id="));
  Serial.println(FIRMWARE_BUILD_ID);
  Serial.println(
    F("FIRMWARE,features=RX350,MODE_KEEP,SERVO_HOLD_2000,SURGE_CAL")
  );
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

  actuatorOutputsBegin();

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
  const uint32_t now = millis();

#if RX315_BENCH_TEST_ONLY
  updateControlManager(now);
  sbusOutputUpdate();
  serialConsoleUpdate();
  return;
#endif

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

  updateControlManager(now);
  sbusOutputUpdate();
  serialConsoleUpdate();
  updateActuatorOutputs(now);
}

void serialEvent1() {
  while (Serial1.available()) {
    packet_decode((uint8_t)Serial1.read());
  }
}
