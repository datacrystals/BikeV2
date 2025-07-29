#include <SPI.h>
#include <mcp2515.h>

// Corrected CAN IDs
#define BMS_CCS_ID 0x1806E5F4  // BMS->Charger command
#define CCS_BCA_ID 0x18FF50E5  // Charger broadcast

// Hardware definitions
#define CAN_CS_PIN 10
#define CAN_INT_PIN 2

// Threshold values (adjust these as needed)
#define CURRENT_THRESHOLD 2.0    // 2.0A
#define VOLTAGE_THRESHOLD 1090   // 109.0V (value is 10x)
#define VOLTAGE_START_THRESHOLD 1060 // 106.0v (value is 10x)


struct can_frame canMsg;
MCP2515 mcp2515(CAN_CS_PIN);

// Charger state
bool chargingActive = true;
uint16_t targetVoltage = 1082;  // 108.2V
uint16_t targetCurrent = 220;   // 22.0A

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for serial connection
  
  SPI.begin();
  
  // Initialize CAN controller
  mcp2515.reset();
  mcp2515.setBitrate(CAN_250KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
  
  pinMode(CAN_INT_PIN, INPUT);
  
  Serial.println("Charger Control System Ready");
  Serial.println("----------------------------");
}

void loop() {
  // Send control command based on current state
  sendChargerControl(targetVoltage, targetCurrent, chargingActive);
  
  // Check for responses and monitor thresholds
  checkForChargerResponse();
  
  delay(1000); // Maintain 1Hz update rate
}

void sendChargerControl(uint16_t voltage_10x, uint16_t current_10x, bool start) {
  // Extended frame with correct ID
  canMsg.can_id = BMS_CCS_ID | 0x80000000;
  canMsg.can_dlc = 8;
  
  // Voltage (big-endian)
  canMsg.data[0] = voltage_10x >> 8;   // High byte
  canMsg.data[1] = voltage_10x & 0xFF;  // Low byte
  
  // Current (big-endian)
  canMsg.data[2] = current_10x >> 8;    // High byte
  canMsg.data[3] = current_10x & 0xFF;  // Low byte
  
  // Control byte - CRITICAL FIX
  canMsg.data[4] = start ? 0x00 : 0x01; // 0=start, 1=stop
  
  // Reserved bytes
  canMsg.data[5] = 0x00;
  canMsg.data[6] = 0x00;
  canMsg.data[7] = 0x00;

  // Send message
  if (mcp2515.sendMessage(&canMsg) == MCP2515::ERROR_OK) {
    Serial.print("CMD: ");
    Serial.print(voltage_10x/10.0, 1); 
    Serial.print("V, ");
    Serial.print(current_10x/10.0, 1);
    Serial.print("A, ");
    Serial.println(start ? "ENABLED" : "DISABLED");
  }
}

void checkForChargerResponse() {
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    // Check if it's charger broadcast
    if ((canMsg.can_id & 0x1FFFFFFF) == CCS_BCA_ID) {
      decodeChargerStatus(canMsg);
    }
  }
}

void decodeChargerStatus(struct can_frame msg) {
  uint16_t voltage = (msg.data[0] << 8) | msg.data[1];
  uint16_t current = (msg.data[2] << 8) | msg.data[3];
  uint8_t status = msg.data[4];
  
  // Extract critical flags
  bool startState = !(status & 0x08); // Bit3: 0=ON, 1=OFF (inverted logic)
  bool commTimeout = (status & 0x10); // Bit4: 1=timeout
  
  Serial.println("----- CHARGER STATUS -----");
  Serial.print("Output: ");
  Serial.print(voltage/10.0, 1);
  Serial.print("V, ");
  Serial.print(current/10.0, 1);
  Serial.println("A");
  
  Serial.print("Start State: ");
  Serial.println(startState ? "ON" : "OFF");
  
  Serial.print("Comm Status: ");
  Serial.println(commTimeout ? "TIMEOUT" : "OK");
  
  Serial.print("Fault Flags: ");
  if (status & 0x01) Serial.print("HARDWARE_FAIL ");
  if (status & 0x02) Serial.print("OVERTEMP ");
  if (status & 0x04) Serial.print("INPUT_VOLTAGE ");
  Serial.println("\n--------------------------");
  
  // Check thresholds only if charging is active
  if (chargingActive) {
    // Check if current has dropped below threshold
    if (current < (CURRENT_THRESHOLD * 10)) {
      Serial.println("ALERT: Current below threshold - stopping charge");
      chargingActive = false;
    }
    
    // Check if voltage has exceeded threshold
    if (voltage > VOLTAGE_THRESHOLD) {
      Serial.println("ALERT: Voltage above threshold - stopping charge");
      chargingActive = false;
    }
  }
  
  if (!chargingActive && voltage > 600 && voltage < VOLTAGE_START_THRESHOLD) {
    Serial.println("Alert: Starting charge, voltage is under starting threshold");
    chargingActive = true;
  }
  
  // Diagnostic advice based on status
  if (!startState) {
    if (voltage < 100) { // 10.0V
      // Serial.println("ALERT: Voltage setting too low?");
    }
    if (current < 10) { // 1.0A
      // Serial.println("ALERT: Current setting too low?");
    }
    if (status & 0x04) {
      Serial.println("ALERT: Check AC input power!");
    }
  }
}