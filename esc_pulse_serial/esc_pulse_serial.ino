// ESC pulse tester for a XIAO ESP32-C6 (two ESCs).
//
//
// Wiring:
//   ESC 1 signal -> D9 (GPIO20)
//   ESC 2 signal -> D7 (GPIO17)
//   ESC grounds  -> any GND pin on the XIAO
//   ESC red wire (BEC +5V) -> leave UNCONNECTED while the XIAO is powered over USB
//
//
// Commands claude created (serial monitor at 115200, press Enter after each one):
//   1 <us>    set ESC 1 pulse, e.g. "1 1500"
//   2 <us>    set ESC 2 pulse
//   b <us>    set both (a bare number also sets both)
//   f <hz>    PWM frequency, 40-400 (default 50)
//   a [us]    arm both ESCs: send the arming pulse (default NEUTRAL_US) and hold it
//   o         ramp both to NEUTRAL_US, then signal OFF (stops sending pulses)
//   s         print status
//   ?         help
//
// Pulses are clamped to LIMIT_LO_US..LIMIT_HI_US. Output starts OFF.

// ---- Settings ----

const int ESC_PIN[2] = {D9, D7};        // ESC 1 on D9 (GPIO20), ESC 2 on D7 (GPIO17)
const int PWM_BITS = 14;                // PWM resolution, duty goes from 0 to 2^14 - 1

const int LIMIT_LO_US = 1000;           // lowest pulse we'll allow
const int LIMIT_HI_US = 5000;           // highest pulse we'll allow

// The "motor stopped" pulse. It's also what we use to arm the ESCs.
// 1500 is for a reversible ESC. For a one-direction ESC change it to 1000.
const int NEUTRAL_US = 1500;
const uint32_t ARM_HOLD_MS = 2000;      // how long to hold the arming pulse

// Pulses ramp toward the target instead of jumping, so the motor doesn't
// get a sudden kick. Every 20 ms the pulse moves at most this many us.
const float SLEW_US_PER_TICK = 10.0f;
const uint32_t TICK_MS = 20;

// ---- State ----

int freqHz = 50;                        // current PWM frequency

bool  escOn[2]     = {false, false};    // false = no pulses being sent
float targetUs[2]  = {0, 0};            // where we want the pulse to end up
float currentUs[2] = {0, 0};            // what we're actually sending right now
bool  stopPending[2] = {false, false};  // ramping down to neutral, then going silent

// ---- Helpers ----

// Send a pulse of "us" microseconds to one ESC.
// The PWM hardware wants a duty value, so work out what fraction of the
// period the pulse takes up and scale it to the PWM resolution.
void writePulse(int esc, float us) {
  float periodUs = 1000000.0f / freqHz;
  uint32_t maxDuty = (1 << PWM_BITS) - 1;
  uint32_t duty = (uint32_t)(us / periodUs * maxDuty);
  ledcWrite(ESC_PIN[esc], duty);
}

void printHelp() {
  Serial.println("Commands: '1 <us>', '2 <us>', 'b <us>' or bare '<us>' (both), 'a [us]' arm, 'f <hz>', 'o' ramp down + off, 's' status, '?' help");
  Serial.printf("Pulse range %d-%d us. Output is OFF until you send a pulse.\n", LIMIT_LO_US, LIMIT_HI_US);
}

void printStatus() {
  for (int esc = 0; esc < 2; esc++) {
    if (escOn[esc]) {
      Serial.printf("ESC%d: target %.0f us, now %.0f us%s\n", esc + 1, targetUs[esc], currentUs[esc],
                    stopPending[esc] ? " (stopping)" : "");
    } else {
      Serial.printf("ESC%d: OFF\n", esc + 1);
    }
  }
  Serial.printf("PWM %d Hz\n", freqHz);
}

// Ramp every running ESC down to the neutral pulse. loop() cuts the signal
// once each one gets there, so the motor slows smoothly instead of being
// chopped off. ESCs that are already off are left alone.
void startStop() {
  bool anyRunning = false;
  for (int esc = 0; esc < 2; esc++) {
    if (!escOn[esc]) continue;
    targetUs[esc] = NEUTRAL_US;
    stopPending[esc] = true;
    anyRunning = true;
  }
  if (anyRunning) {
    Serial.printf("ramping both to %d us, then signal OFF\n", NEUTRAL_US);
  } else {
    Serial.println("already OFF");
  }
}

// Arm both ESCs: jump straight to the arming pulse (no ramp) and hold it
// long enough for the ESC to notice. The PWM hardware keeps sending pulses
// while we wait. The pulse stays at this value afterwards, so nothing spins
// until you send a higher one.
void armEscs(int us) {
  int armUs = constrain(us, LIMIT_LO_US, LIMIT_HI_US);
  for (int esc = 0; esc < 2; esc++) {
    escOn[esc] = true;
    stopPending[esc] = false;
    targetUs[esc] = armUs;
    currentUs[esc] = armUs;
    writePulse(esc, armUs);
  }
  Serial.printf("arming: holding %d us for %lu ms...\n", armUs, (unsigned long)ARM_HOLD_MS);
  delay(ARM_HOLD_MS);
  Serial.println("armed (watch for the ESC beeps/LED)");
}

// Change the PWM frequency and re-apply whatever each ESC was doing.
void setFrequency(int hz) {
  freqHz = hz;
  for (int esc = 0; esc < 2; esc++) {
    ledcChangeFrequency(ESC_PIN[esc], freqHz, PWM_BITS);
    if (escOn[esc]) {
      writePulse(esc, currentUs[esc]);
    } else {
      ledcWrite(ESC_PIN[esc], 0);
    }
  }
  Serial.printf("PWM frequency %d Hz\n", freqHz);
}

// Set a new target pulse for one ESC (clamped to the allowed range).
void setPulse(int esc, int us) {
  int clamped = constrain(us, LIMIT_LO_US, LIMIT_HI_US);
  if (clamped != us) {
    Serial.printf("(clamped %d -> %d us)\n", us, clamped);
  }

  // First pulse after being off: start right at the target. After that
  // loop() ramps toward it.
  if (!escOn[esc]) {
    escOn[esc] = true;
    currentUs[esc] = clamped;
    writePulse(esc, currentUs[esc]);
  }
  targetUs[esc] = clamped;
  stopPending[esc] = false;             // a new pulse cancels a pending stop

  float periodUs = 1000000.0f / freqHz;
  Serial.printf("ESC%d <- %d us  (%.2f ms, duty %.1f%% at %d Hz)\n",
                esc + 1, clamped, clamped / 1000.0f, clamped / periodUs * 100.0f, freqHz);
}

// ---- Command parsing ----

void handleLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  Serial.printf("> %s\n", line.c_str());

  // Single-word commands
  if (line == "?") { printHelp();   return; }
  if (line == "s") { printStatus(); return; }
  if (line == "o") { startStop();   return; }

  char cmd = line.charAt(0);

  // a  or  a <us>  -> arm (default pulse is NEUTRAL_US)
  if (cmd == 'a') {
    String value = line.substring(1);
    value.trim();
    if (value.length() == 0) {
      armEscs(NEUTRAL_US);
    } else if (value.toInt() == 0) {
      Serial.println("usage: a [us]");   // not a number, don't guess a pulse
    } else {
      armEscs(value.toInt());
    }
    return;
  }

  // f <hz>
  if (cmd == 'f') {
    int hz = line.substring(1).toInt();
    if (hz < 40 || hz > 400) {
      Serial.println("f needs 40-400 Hz");
      return;
    }
    setFrequency(hz);
    return;
  }

  // 1 <us> or 2 <us>  (also accepts "1:1500")
  if (cmd == '1' || cmd == '2') {
    String value = line.substring(1);
    value.replace(":", " ");
    value.trim();
    if (value.length() == 0) {
      Serial.println("usage: 1 <us>");
      return;
    }
    int esc = cmd - '1';                // '1' -> 0, '2' -> 1
    setPulse(esc, value.toInt());
    return;
  }

  // b <us>  -> both ESCs
  if (cmd == 'b') {
    String value = line.substring(1);
    value.trim();
    if (value.length() == 0) {
      Serial.println("usage: b <us>");
      return;
    }
    int us = value.toInt();
    setPulse(0, us);
    setPulse(1, us);
    return;
  }

  // Bare number -> both ESCs
  if (isDigit(cmd)) {
    int us = line.toInt();
    setPulse(0, us);
    setPulse(1, us);
    return;
  }

  Serial.println("unknown command, type ? for help");
}

// ---- Arduino entry points ----

void setup() {
  Serial.begin(115200);

  // Attach PWM to both pins, but send nothing (duty 0) until told to.
  for (int esc = 0; esc < 2; esc++) {
    ledcAttach(ESC_PIN[esc], freqHz, PWM_BITS);
    ledcWrite(ESC_PIN[esc], 0);
  }

  delay(500);
  Serial.println("ESC pulse tester ready (XIAO ESP32-C6, ESC1=D9, ESC2=D7)");
  printHelp();
}

void loop() {
  // Read typed characters; a newline means the command is complete.
  static String inputBuffer;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      handleLine(inputBuffer);
      inputBuffer = "";
    } else {
      inputBuffer += c;
    }
  }

  // Only update the ramp once per tick.
  static uint32_t lastTickMs = 0;
  if (millis() - lastTickMs < TICK_MS) return;
  lastTickMs = millis();

  // Move each active ESC a small step toward its target.
  for (int esc = 0; esc < 2; esc++) {
    if (!escOn[esc]) continue;
    float step = constrain(targetUs[esc] - currentUs[esc], -SLEW_US_PER_TICK, SLEW_US_PER_TICK);
    currentUs[esc] += step;
    writePulse(esc, currentUs[esc]);

    // Finished ramping down after an "o": now go silent.
    if (stopPending[esc] && currentUs[esc] == targetUs[esc]) {
      stopPending[esc] = false;
      escOn[esc] = false;
      ledcWrite(ESC_PIN[esc], 0);
      Serial.printf("ESC%d: signal OFF\n", esc + 1);
    }
  }
}
