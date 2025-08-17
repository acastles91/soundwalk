#pragma once
#include <Arduino.h>

namespace center {

struct Cfg {
  int      power;       // 0..255
  uint16_t pulse_ms;
  uint16_t gap_ms;
  int8_t   bias;        // -127..+127  (+ pushes forward more)
  bool     useBrake;
  uint8_t  brakeDuty;   // 0..255
};

enum class State { Idle, PulseFwd, Gap1, PulseRev, Gap2 };

// lifecycle
void init(const Cfg& defaults);  // set config but do NOT start
void on();                       // start dither
void off();                      // stop (coast)

// run each loop()
void tick();

// config / status
void set_cfg(const Cfg& c);
Cfg  get_cfg();

bool is_on();
void set_log(bool on);

const char* state_name(State s);
void status(Print& out);

} // namespace center
