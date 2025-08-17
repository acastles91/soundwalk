// lib/motion/motion.h
#pragma once
#include <Arduino.h>

namespace motion {

struct Config {
  // Ramp (in microseconds)
  int min_delay_us = 12 * 1000;  // your previous minDelay_us
  int max_delay_us = 55 * 1000;  // your previous maxDelay_us
  float ramp_k = 0.65f;          // decay factor
  uint8_t base_pwm_duty = 150;   // current-limiting duty (0..255)
  bool use_zero_cross = false;   // keep false for now
};

void setup(const Config& cfg = Config{});  // pins/LEDC/timers
void startOpenLoop();                      // begin commutation + ramp
void stop();                               // stop timers / outputs

// Call every loop(); adjusts ramp & timer period smoothly
void tick();

// Optional controls
void setDuty(uint8_t duty);       // change base commutation duty (0..255)
bool isRunning();
bool isAccelerated();             // became "fast" according to threshold

// Debug / info
int  currentDelayMicros();
int  commutationStep();

} // namespace motion
