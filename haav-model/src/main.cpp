#include <Arduino.h>
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>

XboxSeriesXControllerESP32_asukiaaa::Core xboxController("0c:35:26:fd:09:85");

const int ESC_COUNT = 2;
const int ESC_PIN[ESC_COUNT] = {D10, D9};  // left motor, right motor
const int SERVO_PIN = D8;
const int LEFT_MOTOR = 0;
const int RIGHT_MOTOR = 1;
const int PWM_HZ = 50;
const int PWM_BITS = 14;
const float JOYSTICK_DEADZONE = 0.1f;
const float INPUT_SMOOTHING = 0.18f;
const float MOTOR_SLEW_PER_SECOND = 2.5f;
const unsigned long REVERSE_NEUTRAL_MS = 1000;

// Reversible ESC: 1500 us is stopped (and arms the ESC), 1000/2000 us are full reverse/forward.
const int NEUTRAL_US = 1500;
const int RANGE_US = 500;

void writePulse(int pin, int us) {
  ledcWrite(pin,
            (uint32_t)us * PWM_HZ * ((1 << PWM_BITS) - 1) / 1000000);
}

float normalizeJoystick(uint16_t value, bool invert) {
  const float center = XboxControllerNotificationParser::maxJoy / 2.0f;
  float normalized = ((float)value - center) / center;
  if (invert) {
    normalized = -normalized;
  }
  if (fabs(normalized) < JOYSTICK_DEADZONE) {
    return 0;
  }
  return normalized > 0
             ? (normalized - JOYSTICK_DEADZONE) / (1 - JOYSTICK_DEADZONE)
             : (normalized + JOYSTICK_DEADZONE) / (1 - JOYSTICK_DEADZONE);
}

float smoothCommand(float target, float &filtered) {
  filtered += (target - filtered) * INPUT_SMOOTHING;
  return filtered;
}

float rampMotor(float target, float &current, unsigned long &zeroSince,
                unsigned long now, float elapsedSeconds) {
  const bool changingDirection = current != 0 && target != 0 &&
                                 ((current > 0) != (target > 0));
  if (changingDirection) {
    target = 0;
  }

  if (target == 0 && current == 0) {
    if (zeroSince == 0) {
      zeroSince = now;
    }
    return 0;
  } else if (target != 0) {
    if (zeroSince != 0) {
      if (now - zeroSince < REVERSE_NEUTRAL_MS) {
        return 0;
      }
      zeroSince = 0;
    }
  }

  float change = target - current;
  float maxChange = MOTOR_SLEW_PER_SECOND * elapsedSeconds;
  if (fabs(change) <= maxChange) {
    current = target;
  } else {
    current += change > 0 ? maxChange : -maxChange;
  }
  return current;
}

void setup() {
  Serial.begin(115200);
  for (int esc = 0; esc < ESC_COUNT; esc++) {
    ledcAttach(ESC_PIN[esc], PWM_HZ, PWM_BITS);
    writePulse(ESC_PIN[esc], NEUTRAL_US);  // hold neutral from power-up
  }
  ledcAttach(SERVO_PIN, PWM_HZ, PWM_BITS);
  writePulse(SERVO_PIN, NEUTRAL_US);  // stop a continuous-rotation servo

  xboxController.begin();
}

void loop() {
  xboxController.onLoop();

  // The library can get stuck after repeated failed connects; a reboot recovers it.
  if (xboxController.getCountFailedConnection() > 2) {
    ESP.restart();
  }

  // The left stick uses arcade mixing: vertical controls speed and horizontal
  // steering makes one motor faster than the other.
  auto &n = xboxController.xboxNotif;
  bool connected = xboxController.isConnected() && !xboxController.isWaitingForFirstNotification();
  float leftThrottle = 0;
  float rightThrottle = 0;
  if (connected) {
    float forward = normalizeJoystick(n.joyLVert, true);
    float turn = normalizeJoystick(n.joyLHori, false);
    leftThrottle = forward + turn;
    rightThrottle = forward - turn;

    float maxThrottle = max(fabs(leftThrottle), fabs(rightThrottle));
    if (maxThrottle > 1) {
      leftThrottle /= maxThrottle;
      rightThrottle /= maxThrottle;
    }
    if (n.btnLB) {
      leftThrottle = -leftThrottle;
    }
    if (n.btnRB) {
      rightThrottle = -rightThrottle;
    }
  }
  static float currentLeftThrottle = 0;
  static float currentRightThrottle = 0;
  static float filteredLeftThrottle = 0;
  static float filteredRightThrottle = 0;
  static unsigned long leftZeroSince = 0;
  static unsigned long rightZeroSince = 0;
  static unsigned long lastMotorUpdate = millis();
  unsigned long now = millis();
  float elapsedSeconds = min((now - lastMotorUpdate) / 1000.0f, 0.1f);
  lastMotorUpdate = now;

  if (!connected) {
    filteredLeftThrottle = 0;
    filteredRightThrottle = 0;
  } else {
    leftThrottle = smoothCommand(leftThrottle, filteredLeftThrottle);
    rightThrottle = smoothCommand(rightThrottle, filteredRightThrottle);
  }
  leftThrottle = rampMotor(filteredLeftThrottle, currentLeftThrottle,
                           leftZeroSince, now, elapsedSeconds);
  rightThrottle = rampMotor(filteredRightThrottle, currentRightThrottle,
                            rightZeroSince, now, elapsedSeconds);

  int leftPulseUs = NEUTRAL_US + currentLeftThrottle * RANGE_US;
  int rightPulseUs = NEUTRAL_US + currentRightThrottle * RANGE_US;
  writePulse(ESC_PIN[LEFT_MOTOR], leftPulseUs);
  writePulse(ESC_PIN[RIGHT_MOTOR], rightPulseUs);

  static float filteredServoCommand = 0;
  float servoCommand = connected
                          ? normalizeJoystick(n.joyRHori, false)
                          : 0;
  servoCommand = smoothCommand(servoCommand, filteredServoCommand);
  int servoPulseUs = NEUTRAL_US + filteredServoCommand * RANGE_US;
  writePulse(SERVO_PIN, servoPulseUs);

  static unsigned long lastLogAt = 0;
  if (millis() - lastLogAt >= 200) {
    lastLogAt = millis();
    Serial.printf("%s | LX %5u LY %5u | RX %5u | %4d us %4d us S %4d us\n",
                  connected ? "CONNECTED" : "NO CTRL  ",
                  n.joyLHori, n.joyLVert, n.joyRHori,
                  leftPulseUs, rightPulseUs, servoPulseUs);
  }

  delay(10);
}
