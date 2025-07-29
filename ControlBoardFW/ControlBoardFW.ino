#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <mcp2515.h>
#include "J1772Controller.h"
#include "CANController.h"

// LCD Configuration (20x4 with PCF8574T)
LiquidCrystal_I2C lcd(0x27, 20, 4);

// CAN Configuration
#define CAN_CS_PIN 10
MCP2515 mcp2515(CAN_CS_PIN);
CANController canController(mcp2515);

// Analog Inputs
const int VOLTAGE_SETPOINT_PIN = A2;
const int CURRENT_SETPOINT_PIN = A3;
const int CHARGE_ENABLE_PIN = A0;  // Charge enable/disable override
#define VOLTAGE_SENSE_PIN A1 // pin for measuring voltage from battery
#define R1 6000 // resistance of top vdiv resistor in kohm
#define R2 120   // resistance of bottom vdiv resistor in kohm

// Digital Pins
const int J1772_PWM_PIN = 3;    // INT1 (pin 3)
const int ENABLE_PIN = 9;       // EVSE Enable

// Safety Thresholds
const float OVERVOLTAGE_THRESHOLD = 109.5;      // Instant shutdown above this
const float VOLTAGE_START_OFFSET = 1.5;         // Start only when below setpoint - this value
const float LOW_CURRENT_THRESHOLD = 2.0;        // Stop charging below this current (A)
const unsigned long LOW_CURRENT_TIMEOUT = 2000; // Time before stopping (ms)

// Measured voltage
float MeasuredValues[5] = {0.0, 0.0, 0.0, 0.0, 0.0}; // last measurements
int LastMeasuredIndex = 0;
float MeasuredVoltage = 0.0; // voltage of the measured input to board

// Timing for low current detection
unsigned long lowCurrentStartTime = 0;
bool lowCurrentCondition = false;

// J1772 Controller
J1772Controller j1772(J1772_PWM_PIN);

// Charger Control Variables
uint16_t targetVoltage = 1082;   // 108.2V (10x scaling)
uint16_t targetCurrent = 220;    // 22.0A (10x scaling)
bool chargingActive = true;
bool safetyStop = false;          // Added safety stop flag
bool chargeEnabled = true;        // Charge enable from analog pin
bool manualDisable = false;       // is manual switch set to disable charging
bool cycleStarted = false;        // Track if charging cycle has started
bool batteryFullLockout = false;  // Lockout after charging completes

// System Measurements
float actualPower = 0.0;          // Calculated (W)
float energy = 0.0;               // Energy (Wh)
bool commTimeout = false;
float lastChargerUpdate = 0.0;    // Time in ms since last update, lets us update for timeout

// Charging States
enum ChargingState {
  STATE_DETECTING,
  STATE_CONNECTED,
  STATE_CHARGING,
  STATE_BATTERY_FULL,
  STATE_FINISHING,
  STATE_SAFETY_STOP,
  STATE_FAULT,
  STATE_COMM_TIMEOUT,
  STATE_NOT_PRESENT,
  STATE_MANUAL_DISABLE
};
ChargingState chargingState = STATE_DETECTING;

// Timing Control
unsigned long lastCanSend = 0;
unsigned long lastDisplayUpdate = 0;
const unsigned long CAN_INTERVAL = 1000;    // 1Hz update
const unsigned long DISPLAY_INTERVAL = 500; // 2Hz update

// Robust LCD initialization with retries
bool initializeLCD() {
  lcd.init();
  lcd.backlight();
  return true;
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n\nEV Charger Controller Booting...");

  Wire.begin();
  Wire.setClock(5000);
  
  // Initialize I/O
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);
  
  // Initialize LCD
  if (!initializeLCD()) {
    Serial.println("Proceeding without LCD");
  } else {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("EV Charger Control");
    lcd.setCursor(0, 1);
    lcd.print("Initializing...");
  }
  
  // Initialize CAN and J1772
  canController.begin();
  j1772.begin();
  
  // Initialize voltage sensor
  pinMode(VOLTAGE_SENSE_PIN, INPUT);

  Serial.println("System initialization complete");
  if (initializeLCD()) {
    lcd.clear();
  }
}

void loop() {
  unsigned long currentMillis = millis();
  
  // Read charge enable override
  int enableRaw = analogRead(CHARGE_ENABLE_PIN);
  chargeEnabled = !(enableRaw > 512);  // Threshold at ~2.5V (50%)
  manualDisable = (enableRaw > 512);
  
  // Read setpoints from potentiometers
  int voltageRaw = 1024 - analogRead(VOLTAGE_SETPOINT_PIN);
  int currentRaw = 1024 - analogRead(CURRENT_SETPOINT_PIN);
  targetVoltage = map(voltageRaw, 0, 1023, 950, 1080);
  targetCurrent = map(currentRaw, 0, 1023, 40, 180);
  
  // Update measured voltage
  int mvRaw = analogRead(VOLTAGE_SENSE_PIN);
  MeasuredValues[LastMeasuredIndex] = (float(mvRaw) / 204.8) * (float(R1) / float(R2));
  LastMeasuredIndex++;
  LastMeasuredIndex = LastMeasuredIndex % 5;

  MeasuredVoltage = 0.0;
  for (int i = 0; i < 5; i++) {
    MeasuredVoltage += MeasuredValues[i];
  }
  MeasuredVoltage /= 5.;


  // Only run J1772 detection when EVSE is disabled
  if (digitalRead(ENABLE_PIN) == LOW) {
    j1772.enableDetection();
    j1772.update();
  } else {
    j1772.disableDetection();
  }

  // Clear lockout if plug is removed or manual disable is toggled
  if (j1772.getState() == J1772Controller::NOT_PRESENT || 
      (manualDisable && !batteryFullLockout)) {
    batteryFullLockout = false;
    // Serial.println("Battery full lockout cleared");
  }

  // Control EVSE enable
  bool prevChargingState = chargingActive;
  
  // Apply safety and enable conditions
  chargingActive = j1772.isPlugPresent() && 
                  (j1772.getMaxCurrent() >= 1.0) && 
                  (targetVoltage >= 1000) && 
                  !safetyStop && 
                  chargeEnabled &&
                  !batteryFullLockout;  // Prevent restart after full charge
  
  // Overvoltage protection using both sources
  if ((canController.getVoltage() > OVERVOLTAGE_THRESHOLD) || 
      (MeasuredVoltage > OVERVOLTAGE_THRESHOLD)) {
    safetyStop = true;
    chargingActive = false;
    Serial.println("SAFETY STOP: Voltage above 109V!");
  }
  else if (safetyStop && 
           canController.getVoltage() < (targetVoltage / 10.0 - VOLTAGE_START_OFFSET) &&
           MeasuredVoltage < (targetVoltage / 10.0 - VOLTAGE_START_OFFSET)) {
    safetyStop = false;  // Clear safety stop only when voltage drops sufficiently
    Serial.println("Safety stop cleared");
  }

  // Start new cycle only if voltage is 1.5V below setpoint and not locked out
  float currentVoltage = canController.getVoltage() > 0 ? canController.getVoltage() : MeasuredVoltage;
  if (!cycleStarted && chargingActive && 
      (currentVoltage < (targetVoltage / 10.0 - VOLTAGE_START_OFFSET))) {
    cycleStarted = true;
    batteryFullLockout = false;  // Clear lockout when starting new cycle
    Serial.println("Starting new charging cycle");
  }
  
  // Stop charging only when current drops below 2A for 2 seconds
  if (cycleStarted && chargingState == STATE_CHARGING && 
      canController.getCurrent() < LOW_CURRENT_THRESHOLD) {
    if (!lowCurrentCondition) {
      lowCurrentStartTime = millis();
      lowCurrentCondition = true;
    } 
    else if (millis() - lowCurrentStartTime >= LOW_CURRENT_TIMEOUT) {
      chargingActive = false;
      cycleStarted = false;
      batteryFullLockout = true;  // Set lockout after charging completes
      chargingState = STATE_FINISHING;
      Serial.println("Low current cutoff activated - lockout set");
    }
  } 
  else {
    lowCurrentCondition = false;
  }

  // Handle finishing state
  if (chargingState == STATE_FINISHING) {
    chargingActive = false;
    // After 5 seconds in finishing state, move to battery full
    if (millis() - lowCurrentStartTime >= 5000) {
      chargingState = STATE_BATTERY_FULL;
      Serial.println("Charging finished");
    }
  }

  if (prevChargingState != chargingActive) {
    Serial.print("EVSE Enable: ");
    Serial.println(chargingActive ? "ENABLED" : "DISABLED");
  }
  digitalWrite(ENABLE_PIN, chargingActive ? HIGH : LOW);
  
  // Send CAN commands at fixed interval
  if (currentMillis - lastCanSend >= CAN_INTERVAL) {
    canController.sendControl(targetVoltage, targetCurrent, true, safetyStop);
    lastCanSend = currentMillis;
  }
  
  // Check for incoming CAN messages
  if (canController.checkForMessages()) {
    lastChargerUpdate = millis();
  }
  
  // Update charging state
  updateChargingState();
  
  // Update display more frequently
  if (currentMillis - lastDisplayUpdate >= DISPLAY_INTERVAL) {

    float actualVoltage = canController.getVoltage();
    float actualCurrent = canController.getCurrent();
    
    if (actualVoltage > 0 && actualCurrent > 0) {
      // Calculate power and accumulate energy
      actualPower = actualVoltage * actualCurrent;
      if (canController.outputState() && chargingActive) {
        energy += actualPower * (DISPLAY_INTERVAL / 3600000.0);  // Convert to Wh
      }
    }

    if (chargingState != STATE_CHARGING) { // reset counters
      actualPower = 0.0;
      energy = 0.0;
    }
    
    updateDisplay();
    lastDisplayUpdate = currentMillis;
  }
}


// ============================= Updated Display Function =============================
void updateDisplay() {
  // Clear display
  lcd.clear();

  // Line 0: State information
  lcd.setCursor(0, 0);
  lcd.print("State: ");
  lcd.print(getStateDescription());
  
  // Line 1: Setpoints and enable status
  lcd.setCursor(0, 1);
  lcd.print("S");
  lcd.print(targetVoltage / 10.0, 1);
  lcd.print("V ");
  lcd.print(targetCurrent / 10.0, 1);
  lcd.print("A");

  // Add J1772 status if space allows
  if (j1772.isPlugPresent()) {
    lcd.setCursor(15, 1);
    lcd.print("J:");
    lcd.print(j1772.getMaxCurrent(), 0);
    lcd.print("A");
  } else {
    lcd.setCursor(15, 1);
    lcd.print("J:**A");
  }

  // Line 2: Reported measurements
  lcd.setCursor(0, 2);
  lcd.print("R");
  float actualVoltage = canController.getVoltage();
  float actualCurrent = canController.getCurrent();
  
  if (actualVoltage > 0 && actualCurrent > 0) {
    lcd.print(actualVoltage, 1);
    lcd.print("V ");
    lcd.print(actualCurrent, 1);
    lcd.print("A");
  } else {
    lcd.print("  ***");
    lcd.print("V ");
    lcd.print("***");
    lcd.print("A");
  }

  
  // Line 3: Power, energy, or fault information
  lcd.setCursor(0, 3);
  
  // Check if we have faults to display
  if (chargingState != STATE_MANUAL_DISABLE) {
    String faultDesc = canController.getFaultDescription();
    if (!canController.isCommTimeout() && faultDesc.length() > 0) {
      // Display fault information
      lcd.print("FAULT: ");
      if (faultDesc.length() <= 13) {
        lcd.print(faultDesc);
      } else {
        // Truncate long fault descriptions
        lcd.print(faultDesc.substring(0, 13));
      }
    } else if (!canController.isCommTimeout()) {
      // Display normal power/energy info
      lcd.print("P:");
      lcd.print(actualPower, 0);
      lcd.print("W E:");
      lcd.print(energy, 0);
      lcd.print("Wh");

      // Print actual measured batt voltage
      lcd.setCursor(15, 3);
      lcd.print(MeasuredVoltage, 1);

    } else {
      lcd.print("TrBatt:");
      lcd.print(MeasuredVoltage, 1);
      lcd.print("V");
    }
  } else {
    lcd.print("TrBatt:");
    lcd.print(MeasuredVoltage, 1);
    lcd.print("V");
  }
}

// ============================= Updated Charging State Logic =============================
void updateChargingState() {
  String faultDesc = canController.getFaultDescription();
  
  if (safetyStop) {
    chargingState = STATE_SAFETY_STOP;
  } 
  else if (manualDisable) {
    chargingState = STATE_MANUAL_DISABLE;
  }
  else if (!chargeEnabled) {
    chargingState = STATE_BATTERY_FULL;
  }
  else if (j1772.getState() == J1772Controller::NOT_PRESENT || 
           j1772.getState() == J1772Controller::INSERTION_TIMEOUT_GATE) {
    chargingState = STATE_NOT_PRESENT;
  } 
  else if (j1772.getState() == J1772Controller::DETECTION_PHASE) {
    chargingState = STATE_DETECTING;
  }
  else if (chargingState == STATE_FINISHING) {
    // Keep in finishing state until timeout
  }
  else if (batteryFullLockout) {
    chargingState = STATE_BATTERY_FULL;
  }
  else if (canController.outputState() && chargingActive && cycleStarted) {
    chargingState = STATE_CHARGING;
  } 
  else if (j1772.isPlugPresent() && cycleStarted) {
    chargingState = STATE_CONNECTED;
  }
  else if (canController.getVoltage() >= (targetVoltage / 10.0 - VOLTAGE_START_OFFSET)) {
    chargingState = STATE_BATTERY_FULL;
  }
  else if (canController.isCommTimeout()) {
    chargingState = STATE_COMM_TIMEOUT;
  }
  else if (faultDesc.length() > 0) {
    chargingState = STATE_FAULT;
  }
  else {
    chargingState = STATE_CONNECTED;
  }
}

// ============================= Updated State Description =============================
const char* getStateDescription() {
  switch (chargingState) {
    case STATE_MANUAL_DISABLE: return "Local Disable";
    case STATE_NOT_PRESENT:    return "No Plug";
    case STATE_DETECTING:      return "Detecting...";
    case STATE_CONNECTED:      return "Plug Connected";
    case STATE_CHARGING:       return "Charging";
    case STATE_FINISHING:      return "Finishing";
    case STATE_BATTERY_FULL:   return "Battery Full";
    case STATE_SAFETY_STOP:    return "E_V_PROT";
    case STATE_FAULT:          return "Fault!";
    case STATE_COMM_TIMEOUT:   return "Comm Timeout";
    default:                   return "Unknown";
  }
}