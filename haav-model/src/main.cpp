#include <Arduino.h>
// Uncomment for the library's own verbose BLE logging (scan results, retries, raw packets)
// #define XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL Serial
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>

// Connects to any Xbox controller in pairing mode.
// To lock to one controller, pass its MAC: xboxController("44:16:22:5e:b2:d4");
XboxSeriesXControllerESP32_asukiaaa::Core xboxController("0c:35:26:fd:09:85");

enum class LinkState { Idle, Scanning, WaitingForFirstNotification, Connected };

const char *linkStateName(LinkState s) {
  switch (s) {
    case LinkState::Idle: return "IDLE (connecting / between scans)";
    case LinkState::Scanning: return "SCANNING (hold pair button until logo flashes fast)";
    case LinkState::WaitingForFirstNotification: return "CONNECTED, waiting for first input packet";
    case LinkState::Connected: return "CONNECTED, receiving input";
  }
  return "?";
}

LinkState readLinkState() {
  if (xboxController.isConnected()) {
    return xboxController.isWaitingForFirstNotification() ? LinkState::WaitingForFirstNotification
                                                          : LinkState::Connected;
  }
  return NimBLEDevice::getScan()->isScanning() ? LinkState::Scanning : LinkState::Idle;
}

LinkState lastState = LinkState::Idle;
uint8_t lastFailCount = 0;
unsigned long lastPrintAt = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);  // give USB serial time to attach so early logs aren't lost
  Serial.println("[xbox] Starting NimBLE client");
  xboxController.begin();
  Serial.println("[xbox] BLE initialized");
}

void loop() {
  xboxController.onLoop();

  LinkState state = readLinkState();
  if (state != lastState) {
    Serial.printf("[xbox] %lu ms: %s -> %s\n", millis(), linkStateName(lastState), linkStateName(state));
    if (state == LinkState::Connected) {
      Serial.printf("[xbox] Controller address: %s\n", xboxController.buildDeviceAddressStr().c_str());
    }
    lastState = state;
  }

  uint8_t failCount = xboxController.getCountFailedConnection();
  if (failCount != lastFailCount) {
    Serial.printf("[xbox] Failed connection attempts: %u\n", failCount);
    lastFailCount = failCount;
  }
  if (failCount > 2) {
    Serial.println("[xbox] Too many failed attempts, restarting ESP32");
    delay(100);
    ESP.restart();
  }

  if (state == LinkState::Connected && millis() - lastPrintAt >= 200) {
    lastPrintAt = millis();
    auto &n = xboxController.xboxNotif;
    const float joyMax = XboxControllerNotificationParser::maxJoy;
    const float trigMax = XboxControllerNotificationParser::maxTrig;

    Serial.printf("LX %.2f LY %.2f RX %.2f RY %.2f LT %.2f RT %.2f A%d B%d X%d Y%d | last packet %lu ms ago\n",
                  n.joyLHori / joyMax, n.joyLVert / joyMax,
                  n.joyRHori / joyMax, n.joyRVert / joyMax,
                  n.trigLT / trigMax, n.trigRT / trigMax,
                  n.btnA, n.btnB, n.btnX, n.btnY,
                  millis() - xboxController.getReceiveNotificationAt());
  }

  delay(10);
}
