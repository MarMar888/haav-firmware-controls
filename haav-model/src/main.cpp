#include <Arduino.h>
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>

XboxSeriesXControllerESP32_asukiaaa::Core xboxController("0c:35:26:fd:09:85");

const int ESC_COUNT = 2;
const int ESC_PIN[ESC_COUNT] = {D10, D9};  // ESC index = LEDC channel
const int PWM_HZ = 50;
const int PWM_BITS = 14;

// Reversible ESC: 1500 us is stopped (and arms the ESC), 1000/2000 us are full reverse/forward.
const int NEUTRAL_US = 1500;
const int RANGE_US = 500;

void writePulse(int esc, int us) {
  ledcWrite(esc, (uint32_t)us * PWM_HZ * ((1 << PWM_BITS) - 1) / 1000000);
}

void setup() {
  Serial.begin(115200);
  for (int esc = 0; esc < ESC_COUNT; esc++) {
    ledcSetup(esc, PWM_HZ, PWM_BITS);
    ledcAttachPin(ESC_PIN[esc], esc);
    writePulse(esc, NEUTRAL_US);  // hold neutral from power-up so the ESCs arm
  }

  xboxController.begin();
}

void loop() {
  xboxController.onLoop();

  // The library can get stuck after repeated failed connects; a reboot recovers it.
  if (xboxController.getCountFailedConnection() > 2) {
    ESP.restart();
  }

  // Throttle in [-1, 1]: right trigger forward, left trigger reverse. Stop if the controller drops.
  auto &n = xboxController.xboxNotif;
  bool connected = xboxController.isConnected() && !xboxController.isWaitingForFirstNotification();
  float throttle = 0;
  if (connected) {
    throttle = (float)(n.trigRT - n.trigLT) / XboxControllerNotificationParser::maxTrig;
  }
  int pulseUs = NEUTRAL_US + throttle * RANGE_US;
  for (int esc = 0; esc < ESC_COUNT; esc++) {
    writePulse(esc, pulseUs);  // both ESCs get the same output for now
  }

  static unsigned long lastLogAt = 0;
  if (millis() - lastLogAt >= 200) {
    lastLogAt = millis();
    const char *dir = throttle > 0 ? "FWD" : throttle < 0 ? "REV" : "STOP";
    Serial.printf("%s | RT %4u LT %4u | ESCs %4d us %-4s (%+.0f%%)\n",
                  connected ? "CONNECTED" : "NO CTRL  ", n.trigRT, n.trigLT, pulseUs, dir, throttle * 100);
  }

  delay(10);
}
