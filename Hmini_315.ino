#include "imu_data_decode.h"
#include "packet.h"
#include "ControlState.h"
#include <Wire.h>
#include "MS5837.h"
#include <SPI.h>
#include <SD.h>
#include <Servo.h>

/*
 * 当前保持为台架接收和SBUS转发测试模式：
 * - 初始化 USB、315 MHz 接收器和Serial2 SBUS输出；
 * - 运行模式、通道映射、入水联锁和USB控制预览；
 * - 仅在AIR模式将315的10个实时通道转发到PX4；
 * - 不初始化传感器、SD、ESC 或舵机；
 *
 * 完成动力安全检查前不要改为 0。
 */
#define RX315_BENCH_TEST_ONLY 1

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

void SDwrite() {
  if (dataFile) {
    dataFile.println(SDdata);
  }
  SDdata = "";
}

void writeNeutralOutputs() {
  esc1.writeMicroseconds(1490);
  esc2.writeMicroseconds(1490);
  esc3.writeMicroseconds(1490);
  esc4.writeMicroseconds(1490);
  servo_left.writeMicroseconds(1000);
  servo_right.writeMicroseconds(2000);
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
  rx315Begin();
  sbusOutputBegin();
  Serial.println(F("CONSOLE_READY,type=help"));

#if RX315_BENCH_TEST_ONLY
  return;
#endif

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

  pinMode(A10, OUTPUT);
  pinMode(A11, OUTPUT);
  digitalWrite(A10, LOW);
  digitalWrite(A11, LOW);

  // 四个水桨：前、后、左、右。
  esc1.attach(8);
  esc2.attach(10);
  esc3.attach(7);
  esc4.attach(9);

  servo_left.attach(4, 544, 2400);
  servo_right.attach(6, 544, 2400);
  writeNeutralOutputs();
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
        Depth = new_depth;
        Temp = new_temp;
        depth_data_valid = true;
        depth_last_update_ms = now;
      } else {
        depth_data_valid = false;
      }
    }
  }

  if (now - time_now >= 50) {
    time_now = now;

    const bool imu_fresh = imuDataFresh(now);
    const bool depth_fresh = depthDataFresh(now);

    if (waterControlCommandAllowed() && imu_fresh && depth_fresh) {
      Marine_Remote_315();
    } else {
      writeNeutralOutputs();
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
