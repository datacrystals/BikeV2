#include "CANController.h"
#include <SPI.h>

CANController::CANController(MCP2515& mcp) 
  : mcp2515(mcp),
    actualVoltage(0.0),
    actualCurrent(0.0),
    chargerOutputState(false),
    commTimeout(false),
    faultFlags(0),
    lastUpdateTime(0) {}

void CANController::begin() {
  SPI.begin();
  mcp2515.reset();
  if (mcp2515.setBitrate(CAN_250KBPS, MCP_8MHZ) != MCP2515::ERROR_OK) {
    Serial.println(F("CAN bitrate configuration failed!"));
  }
  mcp2515.setNormalMode();
  Serial.println("CAN Controller initialized");
}

void CANController::sendControl(uint16_t targetVoltage, uint16_t targetCurrent, bool chargingActive, bool safetyStop) {
  struct can_frame canMsg;
  canMsg.can_id = 0x1806E5F4 | 0x80000000;  // Extended frame
  canMsg.can_dlc = 8;
  
  // Voltage (big-endian)
  canMsg.data[0] = targetVoltage >> 8;
  canMsg.data[1] = targetVoltage & 0xFF;
  
  // Current (big-endian)
  canMsg.data[2] = targetCurrent >> 8;
  canMsg.data[3] = targetCurrent & 0xFF;
  
  // Control byte
  canMsg.data[4] = (chargingActive && !safetyStop) ? 0x00 : 0x01;
  canMsg.data[5] = 0x00;
  canMsg.data[6] = 0x00;
  canMsg.data[7] = 0x00;

  if (mcp2515.sendMessage(&canMsg) == MCP2515::ERROR_OK) {
    Serial.print("CAN Sent: V=");
    Serial.print(targetVoltage/10.0, 1);
    Serial.print("V, I=");
    Serial.print(targetCurrent/10.0, 1);
    Serial.print("A, State=");
    Serial.println((chargingActive && !safetyStop) ? "ON" : "OFF");
  } else {
    Serial.println("CAN Send Failed!");
  }
}

bool CANController::checkForMessages() {
  struct can_frame canMsg;
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    if ((canMsg.can_id & 0x1FFFFFFF) == 0x18FF50E5) {
      actualVoltage = ((canMsg.data[0] << 8) | canMsg.data[1]) / 10.0;
      actualCurrent = ((canMsg.data[2] << 8) | canMsg.data[3]) / 10.0;
      uint8_t status = canMsg.data[4];
      
      chargerOutputState = !(status & 0x08);
      commTimeout = (status & 0x10);
      faultFlags = status & 0x07;
      
      Serial.print("CAN Received: V=");
      Serial.print(actualVoltage, 1);
      Serial.print("V, I=");
      Serial.print(actualCurrent, 1);
      Serial.print("A, Output=");
      Serial.print(chargerOutputState ? "ON" : "OFF");
      Serial.print(", Faults=0x");
      Serial.print(faultFlags, HEX);
      Serial.print(", Comm=");
      Serial.println(commTimeout ? "TIMEOUT" : "OK");

      lastUpdateTime = millis();
      return true;
    }
  }
  return false;
}

float CANController::getVoltage() const { 
  return actualVoltage; 
}

float CANController::getCurrent() const { 
  return actualCurrent; 
}

bool CANController::outputState() const { 
  return chargerOutputState; 
}

bool CANController::isCommTimeout() const { 
  return (millis() - lastUpdateTime) > COMM_TIMEOUT_THRESHOLD; 
}

uint8_t CANController::getFaultFlags() const { 
  return faultFlags; 
}

String CANController::getFaultDescription() const {
  if (faultFlags == 0) return "";
  
  String faultStr = "";
  if (faultFlags & 0x01) faultStr += "HARDWARE_FAIL ";
  if (faultFlags & 0x02) faultStr += "OVERTEMP ";
  if (faultFlags & 0x04) faultStr += "INPUT_VOLTAGE ";
  
  // Remove trailing space
  if (faultStr.length() > 0) {
    faultStr.remove(faultStr.length() - 1);
  }
  
  return faultStr;
}