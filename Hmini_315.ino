#include "imu_data_decode.h"
#include "packet.h"
#include <Wire.h>
#include "MS5837.h"
#include <SPI.h>
#include <SD.h>
#include <Servo.h>

/*
 * 当前保持为台架接收和SBUS转发测试模式：
 * - 初始化 USB、315 MHz 接收器和Serial2 SBUS输出；
 * - 将315的10个通道转发到PX4；
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

int roll_315 = 1500;
int pitch_315 = 1500;
float depth_d_315 = 0;

float phi = 0;
float the = 0;
float psi = 0;
float d_att[4] = {0, 0, 0, 0};

float errSum[4] = {0, 0, 0, 0};
unsigned long time_now = 0;

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

void setup() {
  Serial.begin(115200);
  rx315Begin();
  sbusOutputBegin();

#if RX315_BENCH_TEST_ONLY
  return;
#endif

  Serial1.begin(115200);
  imu_data_decode_init();

  pinMode(53, OUTPUT);
  if (!SD.begin(53)) {
    Serial.println(F("SD initialization failed!"));
    return;
  }
  dataFile = SD.open(FileName, FILE_WRITE);

  Wire.begin();
  if (!sensor.init()) {
    Serial.println(F("Init failed!"));
    Serial.println(F("Are SDA/SCL connected correctly?"));
    Serial.println(F("Blue Robotics Bar30: White=SDA, Green=SCL"));
  }
  sensor.setModel(MS5837::MS5837_30BA);
  sensor.setFluidDensity(1000);
  sensor.read();
  Depth0 = sensor.depth();

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
  sbusOutputUpdate();

#if RX315_BENCH_TEST_ONLY
  return;
#endif

  phi = id0x91.eul[1];
  the = -id0x91.eul[0];
  psi = id0x91.eul[2];
  d_att[0] = id0x91.gyr[1];
  d_att[1] = -id0x91.gyr[0];
  d_att[2] = id0x91.gyr[2];

  sensor.read_nodelay();
  if (sensor.check_step() == 3) {
    Depth = sensor.depth() - Depth0;
    Temp = sensor.temperature();
  }

  if (millis() - time_now >= 50) {
    time_now = millis();

    /*
     * SBUS转发在sbusOutputUpdate()中独立完成。这里仍未把315通道接入
     * 水桨控制变量，关闭台架模式前必须另外完成动力映射和安全检查。
     */
    if (rx315HasFreshFrame()) {
      Marine_Remote_315();
    } else {
      writeNeutralOutputs();
    }
  }
}

void serialEvent1() {
  while (Serial1.available()) {
    packet_decode((uint8_t)Serial1.read());
  }
}
