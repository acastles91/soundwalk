#pragma once
#include <Arduino.h>

namespace actuator {

enum class Mode : uint8_t { IDLE, MANUAL, FORWARD, REVERSE, COAST, BRAKE, AUTO };

struct AutoConfig {
  // durations (ms)
  uint16_t fwdMinMs   = 300,  fwdMaxMs   = 1300;
  uint16_t revMinMs   = 600,  revMaxMs   = 1000;
  uint16_t coastMinMs = 2500, coastMaxMs = 4500;
  uint16_t brakeMinMs = 2000, brakeMaxMs = 3500;

  // duty defaults
  uint8_t  runDuty    = 120;   // base duty for forward/reverse in AUTO
  uint8_t  brakeDuty  = 255;   // duty used during BRAKE in AUTO

  bool     randomize  = true;  // choose next state/duration randomly
};

// Hardware setup
void setup(int pinA, int pinB, int chA, int chB, int pwm_hz, int pwm_res_bits);

// Manual control (MANUAL mode)
void drive(int16_t power);       // -255..255 (sign = direction). 0 -> coast
void coast();                    // both low
void brake(uint8_t duty = 255);  // both high (or equal PWM) -> dynamic brake
void stop();                     // alias of coast()

// Timed burst (overrides everything briefly)
void burst(int16_t power, uint16_t ms);  // signed power, duration

// Auto state machine
void enableAuto(bool on);
bool isAuto();
void setAutoConfig(const AutoConfig& cfg);
AutoConfig getAutoConfig();

// Housekeeping; call regularly
void tick();

// Status
Mode mode();
int16_t currentPower(); // signed, -255..255

// Optional: change base PWM duty for MANUAL quickly (clamped 0..255)
void setManualDuty(uint8_t duty);

// Debug print
void debugPrint(Stream& s);
} // namespace actuator

