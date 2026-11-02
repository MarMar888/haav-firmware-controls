#include <Arduino.h>
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>

XboxSeriesXControllerESP32_asukiaaa::Core xboxController("0c:35:26:fd:09:85");

// Swap these two pins if the wrong motor responds.
const int LEFT_ESC_PIN = D10;
const int RIGHT_ESC_PIN = D9;
const int SERVO_PIN = D8;
const int PWM_HZ = 50;
const int PWM_BITS = 14;

// Set to true to swap which way the right stick turns the servo.
const bool INVERT_SERVO = false;
// Set these if a motor spins the wrong way.
const bool INVERT_LEFT_MOTOR = false;
const bool INVERT_RIGHT_MOTOR = false;
const float STICK_DEADZONE = 0.1f;
// How far the left trigger must be pulled (0..1) before the stick can drive backward.
const float REVERSE_TRIGGER_THRESHOLD = 0.5f;
// How long a motor sits at neutral when switching direction; the ESC won't reverse without it.
const unsigned long REVERSE_PAUSE_MS = 500;
// Fraction of the way each output moves toward the stick each loop; lower is smoother but laggier.
const float SERVO_SMOOTHING = 0.2f;
const float MOTOR_SMOOTHING = 0.1f;

// Reversible ESCs and the continuous-rotation servo: 1500 us is stopped (and arms the ESCs),
// 1000/2000 us are full speed each way.
const int NEUTRAL_US = 1500;
const int RANGE_US = 500;

void writePulse(int pin, int us) {
  ledcWrite(pin, (uint32_t)us * PWM_HZ * ((1 << PWM_BITS) - 1) / 1000000);
}

// Maps a raw stick axis to -1..1, with small movements near center ignored.
// Values past the deadzone are rescaled so output starts at 0 instead of jumping.
float readStick(uint16_t raw) {
  const float center = XboxControllerNotificationParser::maxJoy / 2.0f;
  float value = (raw - center) / center;
  if (fabs(value) < STICK_DEADZONE) {
    return 0;
  }
  float magnitude = (fabs(value) - STICK_DEADZONE) / (1 - STICK_DEADZONE);
  return value > 0 ? magnitude : -magnitude;
}

// Eases toward the target a little each loop to hide uneven controller updates and jitter.
float smooth(float target, float &current, float factor) {
  current += (target - current) * factor;
  return current;
}

// Turns stick x/y (-1..1) into left/right motor throttles (-1..1).
// Forward: the motor on the outside of the turn gets extra throttle, so full right runs
// only the left motor. Reverse steers like a boat backing up: back-right swings the stern
// right by easing off the right motor, and neither motor runs forward while backing.
void mixDrive(float x, float y, float &left, float &right) {
  if (y >= 0) {
    left = y + max(x, 0.0f);
    right = y + max(-x, 0.0f);
    // Scale both down together so a diagonal keeps its turn instead of clipping at full.
    float biggest = max(left, right);
    if (biggest > 1) {
      left /= biggest;
      right /= biggest;
    }
  } else {
    left = y * (1 - max(-x, 0.0f));
    right = y * (1 - max(x, 0.0f));
  }
}

struct Motor {
  float command = 0;  // smoothed throttle (-1..1) actually sent to the ESC
  unsigned long pauseStartedAt = 0;
};

// Smooths a motor toward its target throttle. When the target flips direction, the motor
// drops to neutral and waits REVERSE_PAUSE_MS so the ESC will accept the new direction.
float updateMotor(float target, Motor &motor) {
  bool flipping = (target > 0 && motor.command < 0) || (target < 0 && motor.command > 0);
  if (flipping) {
    motor.command = 0;
    motor.pauseStartedAt = millis();
  }
  if (millis() - motor.pauseStartedAt < REVERSE_PAUSE_MS) {
    return 0;
  }
  smooth(target, motor.command, MOTOR_SMOOTHING);
  // Snap to zero once it has eased in, so a stop at center already counts as the pause.
  if (fabs(motor.command) < 0.01f) {
    motor.command = 0;
  }
  return motor.command;
}

void setup() {
  Serial.begin(115200);
  ledcAttach(LEFT_ESC_PIN, PWM_HZ, PWM_BITS);
  ledcAttach(RIGHT_ESC_PIN, PWM_HZ, PWM_BITS);
  ledcAttach(SERVO_PIN, PWM_HZ, PWM_BITS);
  // Hold everything at neutral from power-up; the ESCs arm on this signal.
  writePulse(LEFT_ESC_PIN, NEUTRAL_US);
  writePulse(RIGHT_ESC_PIN, NEUTRAL_US);
  writePulse(SERVO_PIN, NEUTRAL_US);
  xboxController.begin();
}

void loop() {
  xboxController.onLoop();

  // The library can get stuck after repeated failed connects; a reboot recovers it.
  if (xboxController.getCountFailedConnection() > 2) {
    ESP.restart();
  }

  bool connected = xboxController.isConnected() && !xboxController.isWaitingForFirstNotification();
  auto &n = xboxController.xboxNotif;

  // Left stick drives both motors; they ease to a stop if the controller drops.
  // Raw Y reads lower when pushed up, so it's negated to make forward positive.
  float driveX = connected ? readStick(n.joyLHori) : 0;
  float driveY = connected ? -readStick(n.joyLVert) : 0;
  // Pulling the stick back does nothing unless the left trigger is held.
  bool reverseAllowed = n.trigLT >= REVERSE_TRIGGER_THRESHOLD * XboxControllerNotificationParser::maxTrig;
  if (driveY < 0 && !reverseAllowed) {
    driveX = 0;
    driveY = 0;
  }
  float leftCommand, rightCommand;
  mixDrive(driveX, driveY, leftCommand, rightCommand);
  if (INVERT_LEFT_MOTOR) {
    leftCommand = -leftCommand;
  }
  if (INVERT_RIGHT_MOTOR) {
    rightCommand = -rightCommand;
  }
  static Motor leftMotor, rightMotor;
  int leftPulseUs = NEUTRAL_US + updateMotor(leftCommand, leftMotor) * RANGE_US;
  int rightPulseUs = NEUTRAL_US + updateMotor(rightCommand, rightMotor) * RANGE_US;
  writePulse(LEFT_ESC_PIN, leftPulseUs);
  writePulse(RIGHT_ESC_PIN, rightPulseUs);

  // Right stick side-to-side sets servo speed and direction; it stops if the controller drops.
  float servoCommand = connected ? readStick(n.joyRHori) : 0;
  if (INVERT_SERVO) {
    servoCommand = -servoCommand;
  }
  static float smoothedServoCommand = 0;
  servoCommand = smooth(servoCommand, smoothedServoCommand, SERVO_SMOOTHING);
  int servoPulseUs = NEUTRAL_US + servoCommand * RANGE_US;
  writePulse(SERVO_PIN, servoPulseUs);

  static unsigned long lastLogAt = 0;
  if (millis() - lastLogAt >= 200) {
    lastLogAt = millis();
    const char *servoState = servoPulseUs > NEUTRAL_US   ? "RIGHT"
                             : servoPulseUs < NEUTRAL_US ? "LEFT"
                                                         : "STOP";
    Serial.printf("%s | LX %5u LY %5u LT %4u | L %4d us R %4d us | RX %5u | servo %-5s %+.2f %4d us\n",
                  connected ? "CONNECTED" : "NO CTRL  ",
                  n.joyLHori, n.joyLVert, n.trigLT, leftPulseUs, rightPulseUs,
                  n.joyRHori, servoState, servoCommand, servoPulseUs);
  }

  delay(10);
}
