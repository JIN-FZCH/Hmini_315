#include <avr/interrupt.h>

/*
 * Standalone RX315 diagnostic sketch for Arduino Mega 2560.
 * Stored under tools so it is versioned but not compiled with Hmini_315.
 *
 * Select one mode, upload this sketch separately, and use a 115200-baud
 * Serial Monitor. This sketch is not part of the robot production build.
 */
#define RX315_DIAGNOSTIC_RAW_CAPTURE 1
#define RX315_DIAGNOSTIC_UART_PROBE  2
#define RX315_DIAGNOSTIC_MODE RX315_DIAGNOSTIC_RAW_CAPTURE

#define RX315_CAPTURE_SAMPLES 160
#define RX315_CAPTURE_TIMEOUT_MS 2000UL

#if RX315_DIAGNOSTIC_MODE == RX315_DIAGNOSTIC_RAW_CAPTURE
const uint8_t RX315_OUT_PIN = 15;
volatile uint16_t pulse_us[RX315_CAPTURE_SAMPLES];
volatile uint8_t level_after_edge[RX315_CAPTURE_SAMPLES];
volatile uint8_t sample_count = 0;
volatile bool capture_armed = false;
volatile bool capture_complete = false;
volatile bool first_edge_seen = false;
volatile uint32_t last_edge_us = 0;
uint32_t capture_started_ms = 0;

ISR(PCINT1_vect) {
  if (!capture_armed) {
    return;
  }

  const uint32_t now_us = micros();
  const uint8_t level = (PINJ & _BV(PJ0)) ? 1 : 0;

  if (!first_edge_seen) {
    first_edge_seen = true;
    last_edge_us = now_us;
    return;
  }

  const uint32_t width_us = now_us - last_edge_us;
  last_edge_us = now_us;
  pulse_us[sample_count] =
    (width_us > 65535UL) ? 65535U : (uint16_t)width_us;
  level_after_edge[sample_count] = level;
  sample_count++;

  if (sample_count >= RX315_CAPTURE_SAMPLES) {
    capture_armed = false;
    capture_complete = true;
  }
}

void startCapture() {
  noInterrupts();
  sample_count = 0;
  first_edge_seen = false;
  capture_complete = false;
  capture_armed = true;
  PCIFR |= _BV(PCIF1);
  interrupts();

  capture_started_ms = millis();
  Serial.println(F("RX315_CAPTURE,armed=1,samples=160,timeout_ms=2000"));
}

void setupRawCapture() {
  pinMode(RX315_OUT_PIN, INPUT);
  PCMSK1 |= _BV(PCINT9);
  PCIFR |= _BV(PCIF1);
  PCICR |= _BV(PCIE1);
  Serial.println(F("RX315_RAW_MODE,D15=PCINT9,send 'c' to capture"));
}

void updateRawCapture() {
  while (Serial.available()) {
    const char command = (char)Serial.read();
    if (command == 'c' || command == 'C') {
      startCapture();
    }
  }

  if (capture_armed &&
      millis() - capture_started_ms >= RX315_CAPTURE_TIMEOUT_MS) {
    noInterrupts();
    capture_armed = false;
    capture_complete = true;
    interrupts();
  }

  if (!capture_complete) {
    return;
  }

  noInterrupts();
  const uint8_t count = sample_count;
  capture_complete = false;
  interrupts();

  Serial.print(F("RX315_CAPTURE,complete=1,count="));
  Serial.println(count);
  for (uint8_t i = 0; i < count; ++i) {
    Serial.print(F("RX315_PULSE,index="));
    Serial.print(i);
    Serial.print(F(",level_after_edge="));
    Serial.print(level_after_edge[i]);
    Serial.print(F(",width_us="));
    Serial.println(pulse_us[i]);
  }
}
#endif

#if RX315_DIAGNOSTIC_MODE == RX315_DIAGNOSTIC_UART_PROBE
void setupUartProbe() {
  Serial3.begin(9600);
  Serial.println(F("RX315_UART_PROBE,baud=9600,format=8N1,D15=RX3"));
}

void updateUartProbe() {
  if (!Serial3.available()) {
    return;
  }

  uint8_t count = 0;
  Serial.print(F("RX315_UART,hex="));

  while (Serial3.available() && count < 32) {
    const uint8_t value = (uint8_t)Serial3.read();
    if (value < 0x10) {
      Serial.print('0');
    }
    Serial.print(value, HEX);
    Serial.print(' ');
    count++;
  }

  if (count > 0) {
    Serial.println();
  }
}
#endif

void setup() {
  Serial.begin(115200);

#if RX315_DIAGNOSTIC_MODE == RX315_DIAGNOSTIC_RAW_CAPTURE
  setupRawCapture();
#elif RX315_DIAGNOSTIC_MODE == RX315_DIAGNOSTIC_UART_PROBE
  setupUartProbe();
#endif
}

void loop() {
#if RX315_DIAGNOSTIC_MODE == RX315_DIAGNOSTIC_RAW_CAPTURE
  updateRawCapture();
#elif RX315_DIAGNOSTIC_MODE == RX315_DIAGNOSTIC_UART_PROBE
  updateUartProbe();
#endif
}
