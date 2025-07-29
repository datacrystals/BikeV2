#include "J1772Controller.h"

J1772Controller* J1772Controller::instance = nullptr;

J1772Controller::J1772Controller(uint8_t pin) 
  : pwmPin(pin), 
    detectionEnabled(false),
    state(NOT_PRESENT),
    pulseWidth(0),
    newPulse(false),
    interruptOccurred(false),
    lastPulseTime(0),
    firstPresenceMS(0),
    detectionStart(0),
    detectedPulseWidth(0),
    maxCurrent(0.0) {
  instance = this;
}

void J1772Controller::begin() {
  pinMode(pwmPin, INPUT_PULLUP);
  enableDetection();
  Serial.println("J1772 monitoring initialized");
}

void J1772Controller::enableDetection() {
  if (!detectionEnabled) {
    attachInterrupt(digitalPinToInterrupt(pwmPin), isr, CHANGE);
    detectionEnabled = true;
    Serial.println("J1772 detection enabled");
  }
}

void J1772Controller::disableDetection() {
  if (detectionEnabled) {
    detachInterrupt(digitalPinToInterrupt(pwmPin));
    detectionEnabled = false;
    Serial.println("J1772 detection disabled");
  }
}

bool J1772Controller::isDetectionEnabled() const {
  return detectionEnabled;
}

void J1772Controller::reset() {
  state = NOT_PRESENT;
  newPulse = false;
  lastPulseTime = 0;
  firstPresenceMS = 0;
  detectionStart = 0;
  detectedPulseWidth = 0;
  maxCurrent = 0.0;
  enableDetection();
}

void J1772Controller::update() {
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

bool J1772Controller::isPlugPresent() const { 
  return state == PRESENT; 
}

float J1772Controller::getMaxCurrent() const { 
  return maxCurrent; 
}

J1772Controller::State J1772Controller::getState() const { 
  return state; 
}

void J1772Controller::handleInterrupt() {
  static unsigned long lastRise = 0;

  if (digitalRead(pwmPin)) {
    // Rising edge
    lastRise = micros();
  } else if (lastRise > 0) {
    // Falling edge - calculate pulse width
    pulseWidth = micros() - lastRise;
    newPulse = true;
    lastPulseTime = millis();
    interruptOccurred = true;
  }
}

float J1772Controller::pulseToCurrent(unsigned long pulseWidth) {
  if (pulseWidth < 850) {
    return pulseWidth * 0.06;  // 0.6A per 10μs
  } else {
    return (pulseWidth - 640) * 0.25;  // 2.5A per 10μs
  }
}

// Static ISR wrapper
void J1772Controller::isr() {
  if (instance) {
    instance->handleInterrupt();
  }
}