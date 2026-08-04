#include "ControlState.h"

/*
 * Non-blocking USB serial console.
 *
 * Periodic logs are disabled by default. Safety and mode changes are emitted
 * as one-shot EVENT records by the owning modules.
 */

const uint8_t SERIAL_COMMAND_BUFFER_SIZE = 48;
const uint32_t CHANNEL_WATCH_PERIOD_MS = 200UL;
const uint32_t LINK_WATCH_PERIOD_MS = 1000UL;

char serial_command_buffer[SERIAL_COMMAND_BUFFER_SIZE];
uint8_t serial_command_length = 0;
bool serial_command_overflow = false;
bool watch_channels_enabled = false;
bool watch_link_enabled = false;
bool watch_channels_last_fresh = false;
uint32_t watch_channels_last_ms = 0;
uint32_t watch_link_last_ms = 0;

void serialPrintFrameAge() {
  const uint32_t age = rx315LastFrameAgeMs();
  if (age == 0xFFFFFFFFUL) {
    Serial.print(F("NA"));
  } else {
    Serial.print(age);
  }
}

void serialPrintStatus() {
  const uint32_t now = millis();

  Serial.print(F("STATUS,rx="));
  Serial.print(
    rx315HasFreshFrame() ? F("CONNECTED") : F("DISCONNECTED")
  );
  Serial.print(F(",age_ms="));
  serialPrintFrameAge();
  Serial.print(F(",mode="));
  Serial.print(controlModeName(control_mode));
  Serial.print(F(",state="));
  Serial.print(systemControlStateName(system_control_state));
  Serial.print(F(",reason="));
  Serial.print(systemControlStateReason());
  Serial.print(F(",sbus="));
  Serial.print(sbusOutputStateName());
  Serial.print(F(",water="));
  Serial.print(
    waterControlCommandAllowed() ? F("UNLOCKED") : F("LOCKED")
  );
  Serial.print(F(",actuators="));
  Serial.print(
    actuatorOutputsArmed() ? F("ARMED") : F("DISARMED")
  );
  Serial.print(F(",vertical="));
  Serial.print(depth_control_enabled ? F("ACTIVE") : F("OFF"));
  Serial.print(F(",servo_target_us="));
  Serial.print(angle);
  Serial.print(F(",servo_latched_us="));
  Serial.print(servo_motion_target_us);
  Serial.print(F(",servo_output_us="));
  Serial.print(servo_output_us);
  Serial.print(F(",servo_moving="));
  Serial.print(servo_motion_active ? 1 : 0);
  Serial.print(F(",bench="));
#if RX315_BENCH_TEST_ONLY
  Serial.print(1);
  Serial.print(F(",imu=OFF,depth=OFF"));
#else
  Serial.print(0);
  Serial.print(F(",imu="));
  Serial.print(imuDataFresh(now) ? F("FRESH") : F("STALE"));
  Serial.print(F(",depth="));
  Serial.print(depthDataFresh(now) ? F("FRESH") : F("STALE"));
#endif
  Serial.println();
}

void serialPrintChannels() {
  if (!rx315HasFreshFrame()) {
    Serial.print(F("CHANNELS,rx=DISCONNECTED,age_ms="));
    serialPrintFrameAge();
    Serial.println();
    return;
  }

  Serial.print(F("CHANNELS,RSX/YAW="));
  Serial.print(yaw_315);
  Serial.print(F(",RSY/SURGE="));
  Serial.print(pitch_315);
  Serial.print(F(",LSY/DEPTH="));
  Serial.print(rx315Channel(2));
  Serial.print('/');
  Serial.print(depth_d_315, 3);
  Serial.print(F("m,LSX="));
  Serial.print(rx315Channel(3));
  Serial.print(F(",SA="));
  Serial.print(rx315Channel(4));
  Serial.print(F(",SB="));
  Serial.print(rx315Channel(5));
  Serial.print(F(",SC/MODE="));
  Serial.print(rx315Channel(6));
  Serial.print('/');
  Serial.print(controlModeName(control_mode));
  Serial.print(F(",SD="));
  Serial.print(rx315Channel(7));
  Serial.print(F(",SE="));
  Serial.print(rx315Channel(8));
  Serial.print(F(",SI="));
  Serial.print(rx315Channel(9));
  Serial.print(F(",SERVO_REQUEST="));
  Serial.print(angle);
  Serial.print(':');
  Serial.print(3000 - angle);
  Serial.print(F(",SERVO_LATCHED="));
  Serial.print(servo_motion_target_us);
  Serial.print(':');
  Serial.print(3000 - servo_motion_target_us);
  Serial.print(F(",SERVO_OUTPUT="));
  Serial.print(servo_output_us);
  Serial.print(':');
  Serial.print(3000 - servo_output_us);
  Serial.print(F(",SERVO_MOVING="));
  Serial.print(servo_motion_active ? 1 : 0);
  Serial.print(F(",WATER="));
  Serial.print(
    waterControlCommandAllowed() ? F("UNLOCKED") : F("LOCKED")
  );
  Serial.print(F(",ACTUATORS="));
  Serial.print(
    actuatorOutputsArmed() ? F("ARMED") : F("DISARMED")
  );
  Serial.print(F(",VERTICAL="));
  Serial.println(depth_control_enabled ? F("ACTIVE") : F("OFF"));
}

void serialPrintLink() {
  Serial.print(F("LINK,rx="));
  Serial.print(
    rx315HasFreshFrame() ? F("CONNECTED") : F("DISCONNECTED")
  );
  Serial.print(F(",age_ms="));
  serialPrintFrameAge();
  Serial.print(F(",bytes="));
  Serial.print(rx315ByteCount());
  Serial.print(F(",frames_ok="));
  Serial.print(rx315ValidFrameCount());
  Serial.print(F(",crc_err="));
  Serial.print(rx315CrcErrorCount());
  Serial.print(F(",tail_err="));
  Serial.print(rx315TailErrorCount());
  Serial.print(F(",sync_err="));
  Serial.print(rx315SyncErrorCount());
  Serial.print(F(",seq_lost="));
  Serial.print(rx315SequenceLostCount());
  Serial.print(F(",sbus_frames="));
  Serial.print(sbusOutputTotalFrameCount());
  Serial.print(F(",sbus_state="));
  Serial.print(sbusOutputStateName());
  Serial.print(F(",sbus_failsafe="));
  Serial.println(sbusOutputIsFailsafe() ? 1 : 0);
}

void serialPrintHelp() {
  Serial.println(F("COMMANDS"));
  Serial.println(F("  help                 - show this help"));
  Serial.println(F("  status               - print one system snapshot"));
  Serial.println(F("  channels             - print one channel snapshot"));
  Serial.println(F("  watch channels on    - print CHANNELS every 200 ms"));
  Serial.println(F("  watch channels off   - stop periodic CHANNELS"));
  Serial.println(F("  watch link on        - print LINK every 1000 ms"));
  Serial.println(F("  watch link off       - stop periodic LINK"));
  Serial.println(F("  watch all off        - stop all periodic output"));
  Serial.println(F("LEGEND,RSX/RSY=right-stick X/Y,LSX/LSY=left-stick X/Y"));
  Serial.println(F("LEGEND,DEPTH=PWM/metres,MODE=PWM/name,SERVO=SI/left:right"));
}

void serialHandleCommand(char *command) {
  while (*command == ' ') {
    ++command;
  }

  if (strcmp(command, "help") == 0) {
    serialPrintHelp();
  } else if (strcmp(command, "status") == 0) {
    serialPrintStatus();
  } else if (strcmp(command, "channels") == 0) {
    serialPrintChannels();
  } else if (strcmp(command, "watch channels on") == 0) {
    watch_channels_enabled = true;
    watch_channels_last_fresh = rx315HasFreshFrame();
    watch_channels_last_ms = millis();
    Serial.println(F("OK,watch_channels=ON,period_ms=200"));
    serialPrintChannels();
  } else if (strcmp(command, "watch channels off") == 0) {
    watch_channels_enabled = false;
    Serial.println(F("OK,watch_channels=OFF"));
  } else if (strcmp(command, "watch link on") == 0) {
    watch_link_enabled = true;
    watch_link_last_ms = millis();
    Serial.println(F("OK,watch_link=ON,period_ms=1000"));
    serialPrintLink();
  } else if (strcmp(command, "watch link off") == 0) {
    watch_link_enabled = false;
    Serial.println(F("OK,watch_link=OFF"));
  } else if (strcmp(command, "watch all off") == 0) {
    watch_channels_enabled = false;
    watch_link_enabled = false;
    Serial.println(F("OK,watch_channels=OFF,watch_link=OFF"));
  } else if (*command != '\0') {
    Serial.print(F("ERROR,unknown_command="));
    Serial.println(command);
    Serial.println(F("HINT,type=help"));
  }
}

void serialReadCommands() {
  while (Serial.available()) {
    char value = (char)Serial.read();

    if (value == '\r' || value == '\n') {
      if (serial_command_overflow) {
        Serial.println(F("ERROR,command_too_long"));
      } else {
        while (serial_command_length > 0 &&
               serial_command_buffer[serial_command_length - 1] == ' ') {
          --serial_command_length;
        }
        serial_command_buffer[serial_command_length] = '\0';
        serialHandleCommand(serial_command_buffer);
      }
      serial_command_length = 0;
      serial_command_overflow = false;
      continue;
    }

    if (value == '\b' || value == 0x7F) {
      if (serial_command_length > 0) {
        --serial_command_length;
      }
      continue;
    }

    if (value == '\t') {
      value = ' ';
    }
    if (value >= 'A' && value <= 'Z') {
      value = value - 'A' + 'a';
    }
    if (value < 0x20 || value > 0x7E) {
      continue;
    }

    if (value == ' ' &&
        (serial_command_length == 0 ||
         serial_command_buffer[serial_command_length - 1] == ' ')) {
      continue;
    }

    if (serial_command_length + 1 < SERIAL_COMMAND_BUFFER_SIZE) {
      serial_command_buffer[serial_command_length++] = value;
    } else {
      serial_command_overflow = true;
    }
  }
}

void serialUpdateWatchers() {
  const uint32_t now = millis();

  if (watch_channels_enabled) {
    const bool fresh = rx315HasFreshFrame();
    if (fresh != watch_channels_last_fresh) {
      watch_channels_last_fresh = fresh;
      watch_channels_last_ms = now;
      serialPrintChannels();
    } else if (fresh &&
               now - watch_channels_last_ms >=
                 CHANNEL_WATCH_PERIOD_MS) {
      watch_channels_last_ms = now;
      serialPrintChannels();
    }
  }

  if (watch_link_enabled &&
      now - watch_link_last_ms >= LINK_WATCH_PERIOD_MS) {
    watch_link_last_ms = now;
    serialPrintLink();
  }
}

void serialConsoleUpdate() {
  serialReadCommands();
  serialUpdateWatchers();
}
