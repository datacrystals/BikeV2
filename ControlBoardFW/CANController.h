#ifndef CAN_CONTROLLER_H
#define CAN_CONTROLLER_H

#include <Arduino.h>
#include <mcp2515.h>

class CANController {
public:
  CANController(MCP2515& mcp);
  void begin();
  void sendControl(uint16_t targetVoltage, uint16_t targetCurrent, bool chargingActive, bool safetyStop);
  bool checkForMessages();
  float getVoltage() const;
  float getCurrent() const;
  bool outputState() const;
  bool isCommTimeout() const;
  uint8_t getFaultFlags() const;
  String getFaultDescription() const;

private:
  MCP2515& mcp2515;
  float actualVoltage;
  float actualCurrent;
  bool chargerOutputState;
  bool commTimeout;
  uint8_t faultFlags;
  unsigned long lastUpdateTime;

  static const unsigned long COMM_TIMEOUT_THRESHOLD = 3000; // 3s timeout
};

#endif