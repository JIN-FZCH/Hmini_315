/*
 * 315 MHz receiver production module
 *
 * Receiver OUT -> Mega D15 / RX3
 * UART: 9600 baud, 8N1
 * Frame:
 *   00 AD AD AD AD AD FE 10 CRC Data'[12] BD DB BD
 *
 * This module only receives, validates and decodes channels. It does not
 * control motors, implement failsafe, or transmit data on Serial2.
 */

const uint32_t RX315_BAUD = 9600UL;
const uint32_t RX315_FRESH_TIMEOUT_MS = 200UL;
const uint8_t RX315_CHANNEL_COUNT = 10;

const uint8_t RX315_WHITEN[12] = {
  0xBD, 0xDB, 0xBD, 0xDB, 0xBD, 0xDB,
  0xBD, 0xDB, 0xBD, 0xDB, 0xBD, 0xDB
};

enum Rx315ParserState : uint8_t {
  RX315_WAIT_ZERO,
  RX315_WAIT_AD,
  RX315_WAIT_LENGTH,
  RX315_WAIT_PAYLOAD
};

Rx315ParserState rx315_parser_state = RX315_WAIT_ZERO;
uint8_t rx315_ad_count = 0;
uint8_t rx315_payload[16];
uint8_t rx315_payload_index = 0;

uint16_t rx315_raw_channel[RX315_CHANNEL_COUNT] = {0};
uint16_t rx315_pwm_channel[RX315_CHANNEL_COUNT] = {
  1500, 1500, 1500, 1500, 1500,
  1500, 1500, 1500, 1500, 1500
};
uint16_t rx315_sequence = 0;
uint8_t rx315_reserved = 0;

uint32_t rx315_bytes = 0;
uint32_t rx315_valid_frames = 0;
uint32_t rx315_crc_errors = 0;
uint32_t rx315_tail_errors = 0;
uint32_t rx315_sync_errors = 0;
uint32_t rx315_sequence_lost = 0;
uint32_t rx315_last_valid_ms = 0;
uint32_t rx315_last_frame_print_ms = 0;
uint32_t rx315_last_status_print_ms = 0;
bool rx315_has_valid_frame = false;
bool rx315_sequence_initialized = false;
bool rx315_link_connected = false;

uint8_t rx315Crc8(const uint8_t *data, uint8_t length) {
  uint8_t crc = 0;

  while (length--) {
    crc ^= *data++;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x80)
              ? (uint8_t)((crc << 1) ^ 0x31)
              : (uint8_t)(crc << 1);
    }
  }

  return crc;
}

uint32_t rx315ExtractBits(const uint8_t *data,
                          uint8_t start_bit,
                          uint8_t bit_count) {
  const uint8_t byte_index = start_bit / 8;
  const uint8_t bit_offset = start_bit % 8;
  uint32_t raw = data[byte_index];

  if (byte_index + 1 < 12) {
    raw |= (uint32_t)data[byte_index + 1] << 8;
  }
  if (byte_index + 2 < 12) {
    raw |= (uint32_t)data[byte_index + 2] << 16;
  }

  return (raw >> bit_offset) & ((1UL << bit_count) - 1UL);
}

uint16_t rx315Map11BitToPwm(uint16_t value) {
  return (uint16_t)(1000UL + ((uint32_t)value * 1000UL) / 2047UL);
}

uint16_t rx315Map6BitToPwm(uint16_t value) {
  return (uint16_t)(1000UL + ((uint32_t)value * 1000UL) / 63UL);
}

void rx315PrintFrame() {
  Serial.print(F("RX315_VALID,frames="));
  Serial.print(rx315_valid_frames);
  Serial.print(F(",seq="));
  Serial.print(rx315_sequence);
  Serial.print(F(",ch="));

  for (uint8_t i = 0; i < RX315_CHANNEL_COUNT; ++i) {
    Serial.print(rx315_pwm_channel[i]);
    if (i + 1 < RX315_CHANNEL_COUNT) {
      Serial.print(',');
    }
  }

  Serial.print(F(",crc_errors="));
  Serial.print(rx315_crc_errors);
  Serial.print(F(",tail_errors="));
  Serial.print(rx315_tail_errors);
  Serial.print(F(",sync_errors="));
  Serial.println(rx315_sync_errors);
}

void rx315PrintStatus() {
  Serial.print(F("RX315_STATUS,bytes="));
  Serial.print(rx315_bytes);
  Serial.print(F(",valid="));
  Serial.print(rx315_valid_frames);
  Serial.print(F(",crc_errors="));
  Serial.print(rx315_crc_errors);
  Serial.print(F(",tail_errors="));
  Serial.print(rx315_tail_errors);
  Serial.print(F(",sync_errors="));
  Serial.print(rx315_sync_errors);
  Serial.print(F(",seq_lost="));
  Serial.print(rx315_sequence_lost);
  Serial.print(F(",last_age_ms="));

  if (!rx315_has_valid_frame) {
    Serial.println(F("NA"));
  } else {
    Serial.println(millis() - rx315_last_valid_ms);
  }
}

void rx315SetLinkState(bool connected) {
  if (connected == rx315_link_connected) {
    return;
  }

  rx315_link_connected = connected;
  Serial.print(F("RX315_FLAG,"));

  if (connected) {
    Serial.print(F("CONNECTED,frames="));
    Serial.print(rx315_valid_frames);
    Serial.print(F(",seq="));
    Serial.println(rx315_sequence);
  } else {
    Serial.print(F("DISCONNECTED,last_age_ms="));
    Serial.print(millis() - rx315_last_valid_ms);
    Serial.print(F(",valid="));
    Serial.print(rx315_valid_frames);
    Serial.print(F(",crc_errors="));
    Serial.print(rx315_crc_errors);
    Serial.print(F(",tail_errors="));
    Serial.print(rx315_tail_errors);
    Serial.print(F(",sync_errors="));
    Serial.println(rx315_sync_errors);
  }
}

void rx315DecodeData(const uint8_t *wire_data) {
  uint8_t data[12];

  for (uint8_t i = 0; i < 12; ++i) {
    data[i] = wire_data[i] ^ RX315_WHITEN[i];
  }

  rx315_raw_channel[0] = (uint16_t)rx315ExtractBits(data, 0, 11);
  rx315_raw_channel[1] = (uint16_t)rx315ExtractBits(data, 11, 11);
  rx315_raw_channel[2] = (uint16_t)rx315ExtractBits(data, 22, 11);
  rx315_raw_channel[3] = (uint16_t)rx315ExtractBits(data, 33, 11);
  rx315_raw_channel[4] = (uint16_t)rx315ExtractBits(data, 44, 6);
  rx315_raw_channel[5] = (uint16_t)rx315ExtractBits(data, 50, 6);
  rx315_raw_channel[6] = (uint16_t)rx315ExtractBits(data, 56, 6);
  rx315_raw_channel[7] = (uint16_t)rx315ExtractBits(data, 62, 6);
  rx315_raw_channel[8] = (uint16_t)rx315ExtractBits(data, 68, 6);
  rx315_raw_channel[9] = (uint16_t)rx315ExtractBits(data, 74, 6);

  for (uint8_t i = 0; i < 4; ++i) {
    rx315_pwm_channel[i] = rx315Map11BitToPwm(rx315_raw_channel[i]);
  }
  for (uint8_t i = 4; i < RX315_CHANNEL_COUNT; ++i) {
    rx315_pwm_channel[i] = rx315Map6BitToPwm(rx315_raw_channel[i]);
  }

  const uint16_t new_sequence =
    (uint16_t)rx315ExtractBits(data, 80, 11);

  if (rx315_sequence_initialized) {
    const uint16_t expected = (rx315_sequence + 1U) & 0x07FFU;
    const uint16_t difference = (new_sequence - expected) & 0x07FFU;
    if (difference > 0 && difference < 1024) {
      rx315_sequence_lost += difference;
    }
  }

  rx315_sequence = new_sequence;
  rx315_sequence_initialized = true;
  rx315_reserved = (uint8_t)rx315ExtractBits(data, 91, 5);
}

void rx315HandlePayload() {
  const bool tail_ok =
    rx315_payload[13] == 0xBD &&
    rx315_payload[14] == 0xDB &&
    rx315_payload[15] == 0xBD;

  if (!tail_ok) {
    rx315_tail_errors++;
    return;
  }

  if (rx315Crc8(&rx315_payload[1], 12) != rx315_payload[0]) {
    rx315_crc_errors++;
    return;
  }

  rx315DecodeData(&rx315_payload[1]);
  rx315_valid_frames++;
  rx315_last_valid_ms = millis();
  rx315_has_valid_frame = true;
  rx315SetLinkState(true);

  if (millis() - rx315_last_frame_print_ms >= 100) {
    rx315_last_frame_print_ms = millis();
    rx315PrintFrame();
  }
}

void rx315ResetParser(uint8_t current_value) {
  rx315_ad_count = 0;
  rx315_payload_index = 0;
  rx315_parser_state =
    (current_value == 0x00) ? RX315_WAIT_AD : RX315_WAIT_ZERO;
}

void rx315ConsumeByte(uint8_t value) {
  rx315_bytes++;

  switch (rx315_parser_state) {
    case RX315_WAIT_ZERO:
      if (value == 0x00) {
        rx315_ad_count = 0;
        rx315_parser_state = RX315_WAIT_AD;
      }
      break;

    case RX315_WAIT_AD:
      if (value == 0xAD) {
        if (rx315_ad_count < 5) {
          rx315_ad_count++;
        }
      } else if (value == 0xFE && rx315_ad_count >= 3) {
        rx315_parser_state = RX315_WAIT_LENGTH;
      } else {
        rx315_sync_errors++;
        rx315ResetParser(value);
      }
      break;

    case RX315_WAIT_LENGTH:
      if (value == 0x10) {
        rx315_payload_index = 0;
        rx315_parser_state = RX315_WAIT_PAYLOAD;
      } else {
        rx315_sync_errors++;
        rx315ResetParser(value);
      }
      break;

    case RX315_WAIT_PAYLOAD:
      rx315_payload[rx315_payload_index++] = value;
      if (rx315_payload_index == sizeof(rx315_payload)) {
        rx315HandlePayload();
        rx315ResetParser(0xFF);
      }
      break;
  }
}

void rx315Begin() {
  Serial3.begin(RX315_BAUD);
  Serial.println(F("RX315_BEGIN,baud=9600,format=8N1,D15=RX3"));
  Serial.println(F("RX315_BEGIN,receive/decode only; no motors or PX4 TX"));
}

void rx315Update() {
  while (Serial3.available()) {
    rx315ConsumeByte((uint8_t)Serial3.read());
  }

  // Receiver noise may continue after the handset is turned off. Only a
  // recently validated frame keeps the link in the connected state.
  if (!rx315HasFreshFrame()) {
    rx315SetLinkState(false);
  }

  // Do not continuously print parser statistics while disconnected.
  if (rx315_link_connected &&
      millis() - rx315_last_status_print_ms >= 1000) {
    rx315_last_status_print_ms = millis();
    rx315PrintStatus();
  }
}

uint16_t rx315Channel(uint8_t index) {
  if (index >= RX315_CHANNEL_COUNT) {
    return 1500;
  }
  return rx315_pwm_channel[index];
}

uint16_t rx315RawChannel(uint8_t index) {
  if (index >= RX315_CHANNEL_COUNT) {
    return 0;
  }
  return rx315_raw_channel[index];
}

bool rx315HasFreshFrame() {
  return rx315_has_valid_frame &&
         (millis() - rx315_last_valid_ms <= RX315_FRESH_TIMEOUT_MS);
}

uint32_t rx315LastFrameAgeMs() {
  if (!rx315_has_valid_frame) {
    return 0xFFFFFFFFUL;
  }
  return millis() - rx315_last_valid_ms;
}
