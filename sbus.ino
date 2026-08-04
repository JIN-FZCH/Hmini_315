/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2022 Bolder Flight Systems Inc
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the “Software”), to
* deal in the Software without restriction, including without limitation the
* rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
* sell copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*/

#include "sbus.h"
#include "ControlState.h"

/*
 * Arduino Mega Serial2 TX / D16 -> board SOUT -> PX4 RC/SBUS input.
 *
 * The library writes 11-bit SBUS channel codes. These constants encode the
 * local 315 receiver's PWM-style values so PX4 reports 1000/1500/2000 in
 * RC_CHANNELS on this hardware.
 */
bfs::SbusTx sbus_tx(&Serial2);
bfs::SbusData data;

const uint16_t RX315_PWM_MIN = 1000U;
const uint16_t RX315_PWM_STICK_MID = 1500U;
const uint16_t RX315_PWM_AUX_MID = 1492U;
const uint16_t RX315_PWM_MAX = 2000U;

/*
 * PX4 calibration points measured with this board and standard SBUS input:
 *   SBUS 172 -> PX4 982,  SBUS 992 -> PX4 1494,  SBUS 1811 -> PX4 2004.
 * These compensated codes produce PX4 RC_CHANNELS values near 1000/1500/2000.
 */
const uint16_t SBUS_PX4_MIN = 201U;
const uint16_t SBUS_PX4_MID = 1002U;
const uint16_t SBUS_PX4_MAX = 1802U;
const uint32_t SBUS_OUTPUT_PERIOD_MS = 14UL;
const uint16_t SBUS_FRAME_COUNT_MASK = 0x07FFU;

uint32_t sbus_last_output_ms = 0;
uint32_t sbus_output_total_frames = 0;
uint16_t sbus_output_frame_count = 0;

SbusOutputState sbus_output_state = SBUS_OUTPUT_FAILSAFE;

uint16_t sbusEncodePwmForPx4(uint16_t pwm, uint16_t pwm_mid) {
  if (pwm <= RX315_PWM_MIN) {
    return SBUS_PX4_MIN;
  }

  if (pwm >= RX315_PWM_MAX) {
    return SBUS_PX4_MAX;
  }

  if (pwm <= pwm_mid) {
    const uint16_t input_span = pwm_mid - RX315_PWM_MIN;
    const uint16_t output_span = SBUS_PX4_MID - SBUS_PX4_MIN;
    return (uint16_t)(
      SBUS_PX4_MIN +
      ((uint32_t)(pwm - RX315_PWM_MIN) * output_span + input_span / 2U) /
        input_span
    );
  }

  const uint16_t input_span = RX315_PWM_MAX - pwm_mid;
  const uint16_t output_span = SBUS_PX4_MAX - SBUS_PX4_MID;
  return (uint16_t)(
    SBUS_PX4_MID +
    ((uint32_t)(pwm - pwm_mid) * output_span + input_span / 2U) /
      input_span
  );
}

void sbusSetNeutralData() {
  for (uint8_t i = 0; i < data.NUM_CH; ++i) {
    data.ch[i] = SBUS_PX4_MID;
  }

  // Valid safe RC values: roll/pitch/yaw centered and throttle at minimum.
  data.ch[0] = SBUS_PX4_MID;
  data.ch[1] = SBUS_PX4_MID;
  data.ch[2] = SBUS_PX4_MIN;
  data.ch[3] = SBUS_PX4_MID;

  // Put all six used auxiliary channels in their low position.
  for (uint8_t i = 4; i < 10; ++i) {
    data.ch[i] = SBUS_PX4_MIN;
  }

  data.ch17 = false;
  data.ch18 = false;
  data.lost_frame = false;
  data.failsafe = false;
}

void sbusSetFailsafeData() {
  sbusSetNeutralData();
  data.lost_frame = true;
  data.failsafe = true;
}

void sbusSetRx315Data() {
  // Sticks are decoded as 1000/1500/2000 PWM-style values by RX315.ino.
  for (uint8_t i = 0; i < 4; ++i) {
    data.ch[i] = sbusEncodePwmForPx4(
      rx315Channel(i), RX315_PWM_STICK_MID
    );
  }

  // Six-bit auxiliary channels use 1492 as their physical centre position.
  for (uint8_t i = 4; i < 10; ++i) {
    data.ch[i] = sbusEncodePwmForPx4(
      rx315Channel(i), RX315_PWM_AUX_MID
    );
  }

  for (uint8_t i = 10; i < data.NUM_CH; ++i) {
    data.ch[i] = SBUS_PX4_MID;
  }

  data.ch17 = false;
  data.ch18 = false;
  data.lost_frame = false;
  data.failsafe = false;
}

/*
 * Send one complete standard SBUS frame. CH14 carries the 11-bit sequence
 * counter used by the previous custom 315-to-PX4 protocol. It is diagnostic
 * metadata, not a PWM-style control channel.
 */
void sbusTransmitCurrentData() {
  sbus_output_frame_count =
    (sbus_output_frame_count + 1U) & SBUS_FRAME_COUNT_MASK;
  ++sbus_output_total_frames;

  // SBUS channel 14 is data.ch[13] because the array is zero-indexed.
  data.ch[13] = sbus_output_frame_count;
  sbus_tx.data(data);
  sbus_tx.Write();
}

const char *sbusOutputStateName() {
  switch (sbus_output_state) {
    case SBUS_OUTPUT_ACTIVE:
      return "ACTIVE";
    case SBUS_OUTPUT_NEUTRAL:
      return "NEUTRAL";
    default:
      return "FAILSAFE";
  }
}

const char *sbusOutputStateReason(SbusOutputState state) {
  if (state == SBUS_OUTPUT_FAILSAFE) {
    return systemControlStateReason();
  }
  if (state == SBUS_OUTPUT_ACTIVE) {
    return "AIR_MODE";
  }
  return systemControlStateName(system_control_state);
}

void sbusPrintStateEvent(SbusOutputState state) {
  Serial.print(F("EVENT,sbus="));
  Serial.print(sbusOutputStateName());
  Serial.print(F(",reason="));
  Serial.println(sbusOutputStateReason(state));
}

void sbusOutputBegin() {
  sbusSetFailsafeData();
  sbus_output_state = SBUS_OUTPUT_FAILSAFE;
  sbus_tx.Begin();

  // Send a defined failsafe frame immediately instead of uninitialized data.
  sbusTransmitCurrentData();
  sbus_last_output_ms = millis();
  Serial.println(F("EVENT,sbus=FAILSAFE,reason=STARTUP"));
}

void sbusOutputUpdate() {
  const uint32_t now = millis();
  if (now - sbus_last_output_ms < SBUS_OUTPUT_PERIOD_MS) {
    return;
  }
  sbus_last_output_ms = now;

  /*
   * AIR forwards live RC. With a fresh receiver, STOP and WATER send valid
   * neutral/minimum-throttle RC data without setting failsafe. A real
   * receiver loss still sets both SBUS loss flags.
   */
  SbusOutputState next_state;
  if (system_control_state == SYSTEM_STATE_RX_FAILSAFE) {
    next_state = SBUS_OUTPUT_FAILSAFE;
  } else if (controlModeAllowsSbus()) {
    next_state = SBUS_OUTPUT_ACTIVE;
  } else {
    next_state = SBUS_OUTPUT_NEUTRAL;
  }

  if (next_state == SBUS_OUTPUT_ACTIVE) {
    sbusSetRx315Data();
  } else if (next_state == SBUS_OUTPUT_NEUTRAL) {
    sbusSetNeutralData();
  } else {
    sbusSetFailsafeData();
  }

  if (next_state != sbus_output_state) {
    sbus_output_state = next_state;
    sbusPrintStateEvent(sbus_output_state);
  }
  sbusTransmitCurrentData();
}

bool sbusOutputIsFailsafe() {
  return sbus_output_state == SBUS_OUTPUT_FAILSAFE;
}

uint32_t sbusOutputTotalFrameCount() {
  return sbus_output_total_frames;
}
