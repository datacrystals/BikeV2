#ifndef J1772_CONTROLLER_H
#define J1772_CONTROLLER_H

#include <Arduino.h>

class J1772Controller {
public:
  enum State {
    NOT_PRESENT,
    INSERTION_TIMEOUT_GATE,
    DETECTION_PHASE,
    PRESENT
  };

  J1772Controller(uint8_t pin);
  void begin();
  void update();
  bool isPlugPresent() const;
  float getMaxCurrent() const;
  State getState() const;
  void enableDetection();
  void disableDetection();
  bool isDetectionEnabled() const;
  void reset();

private:
  static void isr();
  void handleInterrupt();
  float pulseToCurrent(unsigned long pulseWidth);

  static J1772Controller* instance;
  const uint8_t pwmPin;
  bool detectionEnabled;

  // State variables
  State state;
  volatile unsigned long pulseWidth;
  volatile bool newPulse;
  volatile bool interruptOccurred;
  volatile unsigned long lastPulseTime;
  unsigned long firstPresenceMS;
  unsigned long detectionStart;
  unsigned long detectedPulseWidth;
  float maxCurrent;

  // Timing constants
  static const unsigned long INSERTION_TIMEOUT = 1500;
  static const unsigned long DETECTION_PHASE_DURATION = 1000;
  static const unsigned long SIGNAL_LOSS_TIMEOUT = 2000;
};

#endif