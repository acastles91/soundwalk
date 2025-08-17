#include "center.h"
#include <actuator.h>

namespace center {

static Cfg   s_cfg{ 90, 25, 30, 0, false, 200 };
static bool  s_on  = false;
static bool  s_log = false;

static State     s_state = State::Idle;
static uint32_t  s_tmr   = 0;

static inline void set_state(State ns, uint32_t now) {
  if (ns != s_state && s_log) {
    Serial.printf("CENTER -> %-9s  t=%lu ms\n", state_name(ns), (unsigned long)now);
  }
  s_state = ns;
}

void init(const Cfg& defaults) { s_cfg = defaults; s_on = false; s_state = State::Idle; }

void on() {
  s_on   = true;
  s_state= State::Idle;
  if (s_log) {
    Serial.printf("CENTER ON  power=%d pulse=%u gap=%u bias=%d brake=%s(%u)\n",
      s_cfg.power, s_cfg.pulse_ms, s_cfg.gap_ms, s_cfg.bias,
      s_cfg.useBrake ? "on":"off", s_cfg.brakeDuty);
  }
}

void off() {
  if (s_on && s_log) Serial.println("CENTER OFF");
  s_on   = false;
  s_state= State::Idle;
  actuator::coast();
}

void tick() {
  if (!s_on) return;
  uint32_t now = millis();

  auto brakeOrCoast = [&](){
    if (s_cfg.useBrake) actuator::brake(s_cfg.brakeDuty);
    else                actuator::coast();
  };

  switch (s_state) {
    case State::Idle:
      actuator::drive( constrain(s_cfg.power + max(0, (int)s_cfg.bias), 0, 255) );
      s_tmr = now;
      set_state(State::PulseFwd, now);
      break;

    case State::PulseFwd:
      if (now - s_tmr >= s_cfg.pulse_ms) {
        brakeOrCoast();
        s_tmr = now;
        set_state(State::Gap1, now);
      }
      break;

    case State::Gap1:
      if (now - s_tmr >= s_cfg.gap_ms) {
        actuator::drive( -constrain(s_cfg.power + max(0, -(int)s_cfg.bias), 0, 255) );
        s_tmr = now;
        set_state(State::PulseRev, now);
      }
      break;

    case State::PulseRev:
      if (now - s_tmr >= s_cfg.pulse_ms) {
        brakeOrCoast();
        s_tmr = now;
        set_state(State::Gap2, now);
      }
      break;

    case State::Gap2:
      if (now - s_tmr >= s_cfg.gap_ms) {
        actuator::drive( constrain(s_cfg.power + max(0, (int)s_cfg.bias), 0, 255) );
        s_tmr = now;
        set_state(State::PulseFwd, now);
      }
      break;
  }
}

void set_cfg(const Cfg& c) { s_cfg = c; }
Cfg  get_cfg()             { return s_cfg; }

bool is_on()               { return s_on; }
void set_log(bool on)      { s_log = on; }

const char* state_name(State s){
  switch (s){
    case State::Idle:     return "Idle";
    case State::PulseFwd: return "PulseFwd";
    case State::Gap1:     return "Gap1";
    case State::PulseRev: return "PulseRev";
    case State::Gap2:     return "Gap2";
  }
  return "?";
}

void status(Print& out){
  out.printf("CENTER %s power=%d pulse=%u gap=%u bias=%d brake=%s(%u)\n",
    s_on?"ON":"OFF", s_cfg.power, s_cfg.pulse_ms, s_cfg.gap_ms,
    s_cfg.bias, s_cfg.useBrake?"on":"off", s_cfg.brakeDuty);
}

} // namespace center
