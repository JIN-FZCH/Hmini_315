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

/*
 * Arduino Mega Serial2 TX / D16 -> board SOUT -> PX4 RC/SBUS input.
 *
 * The Bolder Flight Systems library expects channel values in the usual
 * SBUS range (172..1811), not PWM microseconds (1000..2000).
 */
bfs::SbusTx sbus_tx(&Serial2);
bfs::SbusData data;

const uint16_t SBUS_OUTPUT_MIN = 172U;
const uint16_t SBUS_OUTPUT_MID = 992U;
const uint16_t SBUS_OUTPUT_MAX = 1811U;
const uint32_t SBUS_OUTPUT_PERIOD_MS = 14UL;

uint32_t sbus_last_output_ms = 0;

uint16_t sbusMapRawChannel(uint16_t value, uint16_t input_max) {
  if (value > input_max) {
    value = input_max;
  }

  const uint32_t output_span = SBUS_OUTPUT_MAX - SBUS_OUTPUT_MIN;
  return (uint16_t)(
    SBUS_OUTPUT_MIN +
    ((uint32_t)value * output_span + input_max / 2U) / input_max
  );
}

void sbusSetFailsafeData() {
  for (uint8_t i = 0; i < data.NUM_CH; ++i) {
    data.ch[i] = SBUS_OUTPUT_MID;
  }

  // Safe stick values: roll/pitch/yaw centered and throttle at minimum.
  data.ch[0] = SBUS_OUTPUT_MID;
  data.ch[1] = SBUS_OUTPUT_MID;
  data.ch[2] = SBUS_OUTPUT_MIN;
  data.ch[3] = SBUS_OUTPUT_MID;

  // Put all six used auxiliary channels in their low position.
  for (uint8_t i = 4; i < 10; ++i) {
    data.ch[i] = SBUS_OUTPUT_MIN;
  }

  data.ch17 = false;
  data.ch18 = false;
  data.lost_frame = true;
  data.failsafe = true;
}

void sbusSetRx315Data() {
  // The four stick channels have 11-bit source resolution.
  for (uint8_t i = 0; i < 4; ++i) {
    data.ch[i] = sbusMapRawChannel(rx315RawChannel(i), 2047U);
  }

  // The six auxiliary channels have 6-bit source resolution.
  for (uint8_t i = 4; i < 10; ++i) {
    data.ch[i] = sbusMapRawChannel(rx315RawChannel(i), 63U);
  }

  for (uint8_t i = 10; i < data.NUM_CH; ++i) {
    data.ch[i] = SBUS_OUTPUT_MID;
  }

  data.ch17 = false;
  data.ch18 = false;
  data.lost_frame = false;
  data.failsafe = false;
}

void sbusOutputBegin() {
  sbusSetFailsafeData();
  sbus_tx.Begin();

  // Send a defined failsafe frame immediately instead of uninitialized data.
  sbus_tx.data(data);
  sbus_tx.Write();
  sbus_last_output_ms = millis();
}

void sbusOutputUpdate() {
  const uint32_t now = millis();
  if (now - sbus_last_output_ms < SBUS_OUTPUT_PERIOD_MS) {
    return;
  }
  sbus_last_output_ms = now;

  if (rx315HasFreshFrame()) {
    sbusSetRx315Data();
  } else {
    sbusSetFailsafeData();
  }

  sbus_tx.data(data);
  sbus_tx.Write();
}
