#ifndef HMINI_315_CONTROL_STATE_H
#define HMINI_315_CONTROL_STATE_H

#include <Arduino.h>

enum ControlMode : uint8_t {
  CONTROL_MODE_AIR,
  CONTROL_MODE_STOP,
  CONTROL_MODE_WATER
};

enum SbusOutputState : uint8_t {
  SBUS_OUTPUT_ACTIVE,
  SBUS_OUTPUT_NEUTRAL,
  SBUS_OUTPUT_FAILSAFE
};

extern ControlMode control_mode;
extern bool water_entry_safe;
extern bool water_control_unlocked;
extern bool actuator_outputs_armed;
extern bool depth_control_enabled;
extern int servo_output_us;
extern int servo_motion_target_us;
extern bool servo_motion_active;

void updateControlState();
bool controlModeAllowsSbus();
bool waterControlCommandAllowed();
void invalidateWaterControl(const char *reason);
void resetWaterControllerState();
void Marine_Remote_315();
const char *controlModeName(ControlMode mode);

void serialConsoleUpdate();
void serialPrintStatus();
void serialPrintChannels();
void serialPrintLink();

void rx315Begin();
void rx315Update();
void sbusOutputBegin();
void sbusOutputUpdate();

bool imuDataFresh(uint32_t now);
bool depthDataFresh(uint32_t now);
bool actuatorOutputsArmed();
void armActuatorOutputs(const char *reason);
void disarmActuatorOutputs(const char *reason);
void enforceActuatorOutputSafety(uint32_t now);
void TD(float *x, float u);
float fst(float *x, float u, float r, float h);
float sgn(float x);
float *PID_outloop(float error[]);

uint32_t rx315ByteCount();
bool rx315HasFreshFrame();
uint16_t rx315Channel(uint8_t index);
uint32_t rx315ValidFrameCount();
uint32_t rx315CrcErrorCount();
uint32_t rx315TailErrorCount();
uint32_t rx315SyncErrorCount();
uint32_t rx315SequenceLostCount();
uint32_t rx315LastFrameAgeMs();

bool sbusOutputIsFailsafe();
const char *sbusOutputStateName();
uint32_t sbusOutputTotalFrameCount();

#endif
