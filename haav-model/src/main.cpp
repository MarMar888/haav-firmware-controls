#include <Arduino.h>
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>

XboxSeriesXControllerESP32_asukiaaa::Core xboxController("0c:35:26:fd:09:85");

const int ESC_PIN = D10;
const int ESC_CHANNEL = 0;
const int PWM_HZ = 50;
const int PWM_BITS = 14;

// Reversible ESC: 1500 us is stopped (and arms the ESC), 1000/2000 us are full reverse/forward.
const int NEUTRAL_US = 1500;
const int RANGE_US = 500;

void writePulse(int us) {
  ledcWrite(ESC_CHANNEL, (uint32_t)us * PWM_HZ * ((1 << PWM_BITS) - 1) / 1000000);
}

void setup() {
  Serial.begin(115200);
  ledcSetup(ESC_CHANNEL, PWM_HZ, PWM_BITS);
  ledcAttachPin(ESC_PIN, ESC_CHANNEL);
  writePulse(NEUTRAL_US);  // hold neutral from power-up so the ESC arms

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
  writePulse(pulseUs);

  static unsigned long lastLogAt = 0;
  if (millis() - lastLogAt >= 200) {
    lastLogAt = millis();
    const char *dir = throttle > 0 ? "FWD" : throttle < 0 ? "REV" : "STOP";
    Serial.printf("%s | RT %4u LT %4u | ESC %4d us %-4s (%+.0f%%)\n",
                  connected ? "CONNECTED" : "NO CTRL  ", n.trigRT, n.trigLT, pulseUs, dir, throttle * 100);
  }

  delay(10);
}
