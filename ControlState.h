#ifndef HMINI_315_CONTROL_STATE_H
#define HMINI_315_CONTROL_STATE_H

#include <Arduino.h>

enum ControlMode : uint8_t {
  CONTROL_MODE_AIR,
  CONTROL_MODE_SAFE,
  CONTROL_MODE_WATER
};

extern ControlMode control_mode;
extern bool water_entry_safe;
extern bool water_control_unlocked;

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

bool imuDataFresh(uint32_t now);
bool depthDataFresh(uint32_t now);

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
uint32_t sbusOutputTotalFrameCount();

#endif
