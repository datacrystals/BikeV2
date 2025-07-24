#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <mcp2515.h>

// LCD Configuration (20x4 with PCF8574T)
LiquidCrystal_I2C lcd(0x27, 20, 4);

// CAN Configuration
#define BMS_CCS_ID 0x1806E5F4
#define CCS_BCA_ID 0x18FF50E5
#define CAN_CS_PIN 10
#define CAN_INT_PIN 2
MCP2515 mcp2515(CAN_CS_PIN);
struct can_frame canMsg;

// Analog Inputs
const int VOLTAGE_SETPOINT_PIN = A2;
const int CURRENT_SETPOINT_PIN = A3;
const int CHARGE_ENABLE_PIN = A0;  // Charge enable/disable override
#define VOLTAGE_SENSE_PIN A1 // pin for measuring voltage from battery
#define R1 6000 // resistance of top vdiv resistor in kohm
#define R2 60   // resistance of bottom vdiv resistor in kohm

// Digital Pins
const int J1772_PWM_PIN = 3;    // INT1 (pin 3)
const int ENABLE_PIN = 9;       // EVSE Enable

// Safety Thresholds
const float OVERVOLTAGE_THRESHOLD = 109.0;      // Instant shutdown above this
const float VOLTAGE_START_OFFSET = 1.5;         // Start only when below setpoint - this value

// Measured voltage
float MeasuredVoltage = 0.0; // voltage of the measured input to board


// to fix:

// make the charger cut off once the current drops below 2a
// make the j1772 controller only detect presence at the start - once it is done, we dont care anymore. We ONLY re-run detection if the charger has complained about input voltage THATS IT
// in fact, disable the interrupt once we're done with detection, and don't re-enable it unless the charger says input voltage issues, or some other issues.
// seriously, the noise makes it reeset the EVSE every few seconds, and it's really fucking annoying.

// next, add the measured voltage to overvoltage protection, where it will cut off the charger if voltage is measured at OVERVOLTAGE_THRESHOLD

// lastly, add sequencing things, so turning off the charger will order it to stop charging, then you remove the cable, so it's not under load.
// oh and make it so that lowering the voltage adjust knob doesnt make it turn on and off again, it should only start a new cycle if the voltage is 1.5v below the current voltage. then it stops when the current drops below 2a for 2s.


// ============================= J1772 Controller Class =============================
class J1772Controller {
public:
  enum State {
    NOT_PRESENT,
    INSERTION_TIMEOUT_GATE,
    DETECTION_PHASE,
    PRESENT
  };

  J1772Controller(uint8_t pin) : pwmPin(pin) {
    // Initialize the static instance pointer
    instance = this;
  }

  void begin() {
    pinMode(pwmPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(pwmPin), isr, CHANGE);
    Serial.println("J1772 monitoring initialized");
  }

  void update() {
    unsigned long currentMillis = millis();
    
    // State transitions
    switch (state) {
      case NOT_PRESENT:
        if (newPulse) {
          state = INSERTION_TIMEOUT_GATE;
          firstPresenceMS = currentMillis;
          newPulse = false;
          Serial.println("J1772: Plug detected, starting insertion timeout");
        }
        break;
        
      case INSERTION_TIMEOUT_GATE:
        if (currentMillis - firstPresenceMS >= INSERTION_TIMEOUT) {
          state = DETECTION_PHASE;
          detectionStart = currentMillis;
          Serial.println("J1772: Starting detection phase");
        }
        break;
        
      case DETECTION_PHASE:
        if (newPulse) {
          // Record pulse width but don't set detected yet
          detectedPulseWidth = pulseWidth;
          newPulse = false;
        }
        
        // After 1 second, finalize detection
        if (currentMillis - detectionStart >= DETECTION_PHASE_DURATION) {
          if (detectedPulseWidth > 0) {
            maxCurrent = pulseToCurrent(detectedPulseWidth);
            maxCurrent = constrain(maxCurrent, 6.0, 80.0);
            state = PRESENT;
            Serial.print("J1772: Plug present - Max current: ");
            Serial.print(maxCurrent);
            Serial.println("A");
          } else {
            state = NOT_PRESENT;
            Serial.println("J1772: Detection timed out");
          }
        }
        break;
        
      case PRESENT:
        // Check for signal loss
        if (currentMillis - lastPulseTime > SIGNAL_LOSS_TIMEOUT) {
          state = NOT_PRESENT;
          maxCurrent = 0;
          Serial.println("J1772: Signal lost");
        }
        break;
    }
  }

  bool isPlugPresent() const { return state == PRESENT; }
  float getMaxCurrent() const { return maxCurrent; }
  State getState() const { return state; }

  // Interrupt handler
  static void handleInterrupt() {
    static unsigned long lastRise = 0;

    if (digitalRead(instance->pwmPin)) {
      // Rising edge
      lastRise = micros();
    } else if (lastRise > 0) {
      // Falling edge - calculate pulse width
      instance->pulseWidth = micros() - lastRise;
      instance->newPulse = true;
      instance->lastPulseTime = millis();
      
      // For debugging - set a flag instead of using Serial in ISR
      instance->interruptOccurred = true;
    }
  }

  // Call this in loop() to check for interrupts
  bool checkInterruptOccurred() {
    if (interruptOccurred) {
      interruptOccurred = false;
      return true;
    }
    return false;
  }

private:
  static J1772Controller* instance;
  const uint8_t pwmPin;
  
  // State variables
  State state = NOT_PRESENT;
  volatile unsigned long pulseWidth = 0;
  volatile bool newPulse = false;
  volatile bool interruptOccurred = false;
  volatile unsigned long lastPulseTime = 0;
  unsigned long firstPresenceMS = 0;
  unsigned long detectionStart = 0;
  unsigned long detectedPulseWidth = 0;
  float maxCurrent = 0.0;

  // Timing constants
  static const unsigned long INSERTION_TIMEOUT = 1500;      // 1.5s insertion gate
  static const unsigned long DETECTION_PHASE_DURATION = 1000; // 1s detection phase
  static const unsigned long SIGNAL_LOSS_TIMEOUT = 2000;    // 2s signal loss timeout

  // PWM to Current conversion
  float pulseToCurrent(unsigned long pulseWidth) {
    if (pulseWidth < 850) {
      return pulseWidth * 0.06;  // 0.6A per 10μs
    } else {
      return (pulseWidth - 640) * 0.25;  // 2.5A per 10μs
    }
  }

  // Static ISR wrapper
  static void isr() {
    if (instance) {
      instance->handleInterrupt();
    }
  }
};

// Initialize static member
J1772Controller* J1772Controller::instance = nullptr;
J1772Controller j1772(J1772_PWM_PIN);

// ============================= CAN Controller Class =============================
class CANController {
public:
  CANController(MCP2515& mcp) : mcp2515(mcp) {}

  void begin() {
    SPI.begin();
    mcp2515.reset();
    if (mcp2515.setBitrate(CAN_250KBPS, MCP_8MHZ) != MCP2515::ERROR_OK) {
      Serial.println(F("CAN bitrate configuration failed!"));
    }
    mcp2515.setNormalMode();
    Serial.println("CAN Controller initialized");
  }

  void sendControl(uint16_t targetVoltage, uint16_t targetCurrent, bool chargingActive, bool safetyStop) {
    canMsg.can_id = BMS_CCS_ID | 0x80000000;  // Extended frame
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

  bool checkForMessages() {
    if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
      if ((canMsg.can_id & 0x1FFFFFFF) == CCS_BCA_ID) {
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

  float getVoltage() const { return actualVoltage; }
  float getCurrent() const { return actualCurrent; }
  bool outputState() const { return chargerOutputState; }
  bool isCommTimeout() const { 
    return (millis() - lastUpdateTime) > COMM_TIMEOUT_THRESHOLD; 
  }
  uint8_t getFaultFlags() const { return faultFlags; }
  
  // Get human-readable fault description
  String getFaultDescription() const {
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

private:
  MCP2515& mcp2515;
  float actualVoltage = 0.0;
  float actualCurrent = 0.0;
  bool chargerOutputState = false;
  bool commTimeout = false;
  uint8_t faultFlags = 0;
  unsigned long lastUpdateTime = 0;
  
  static const unsigned long COMM_TIMEOUT_THRESHOLD = 3000; // 3s timeout
};

CANController canController(mcp2515);


// ============================= Main Program =============================
// Charger Control Variables
uint16_t targetVoltage = 1082;   // 108.2V (10x scaling)
uint16_t targetCurrent = 220;    // 22.0A (10x scaling)
bool chargingActive = true;
bool safetyStop = false;          // Added safety stop flag
bool chargeEnabled = true;        // Charge enable from analog pin
bool manualDisable = false;       // is manual switch set to disable charging

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
  
  // Update J1772 state machine
  j1772.update();
  
  // Read charge enable override
  int enableRaw = analogRead(CHARGE_ENABLE_PIN);
  chargeEnabled = !(enableRaw > 512);  // Threshold at ~2.5V (50%)
  manualDisable = (enableRaw > 512);
  
  // Read setpoints from potentiometers
  int voltageRaw = 1024 - analogRead(VOLTAGE_SETPOINT_PIN);
  int currentRaw = 1024 - analogRead(CURRENT_SETPOINT_PIN);
  targetVoltage = map(voltageRaw, 0, 1023, 900, 1080);
  targetCurrent = map(currentRaw, 0, 1023, 10, 180);
  
  // Update measured voltage
  int mvRaw = analogRead(VOLTAGE_SENSE_PIN);
  MeasuredVoltage = (float(mvRaw) / 204.8) * (float(R1) / float(R2));

  // Control EVSE enable
  bool prevChargingState = chargingActive;
  
  // Apply safety and enable conditions
  chargingActive = j1772.isPlugPresent() && 
                  (j1772.getMaxCurrent() >= 1.0) && 
                  (targetVoltage >= 1000) && 
                  !safetyStop && 
                  chargeEnabled;
  
  // Add voltage-based start/stop logic (1.5V hysteresis)
  float actualVoltage = canController.getVoltage();
  if (actualVoltage > 0) {  // Only if we have valid voltage reading
    if (actualVoltage > OVERVOLTAGE_THRESHOLD) {
      safetyStop = true;
      chargingActive = false;
      Serial.println("SAFETY STOP: Voltage above 109V!");
    }
    else if (safetyStop && actualVoltage < (targetVoltage / 10.0 - VOLTAGE_START_OFFSET)) {
      safetyStop = false;  // Clear safety stop only when voltage drops sufficiently
      Serial.println("Safety stop cleared");
    }
    
    // Enable charging only if voltage is 1.5V below setpoint
    if (chargingActive && actualVoltage > (targetVoltage / 10.0 - VOLTAGE_START_OFFSET)) {
      chargingActive = false;
      Serial.println("Charging paused: Voltage too close to setpoint");
    }
  }

  if (prevChargingState != chargingActive) {
    Serial.print("EVSE Enable: ");
    Serial.println(chargingActive ? "ENABLED" : "DISABLED");
  }
  digitalWrite(ENABLE_PIN, chargingActive ? HIGH : LOW);
  
  // Send CAN commands at fixed interval
  if (currentMillis - lastCanSend >= CAN_INTERVAL) {
    canController.sendControl(targetVoltage, targetCurrent, chargingActive, safetyStop);
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
      lcd.print(" A");
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
  else if (canController.outputState()) {
    chargingState = STATE_CHARGING;
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
    case STATE_BATTERY_FULL:   return "Battery Full";
    case STATE_SAFETY_STOP:    return "E_V_PROT";
    case STATE_FAULT:          return "Fault!";
    case STATE_COMM_TIMEOUT:   return "Comm Timeout";
    default:                   return "Unknown";
  }
}

