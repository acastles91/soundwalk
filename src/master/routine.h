#pragma once
#include <Arduino.h>

namespace routine {

enum class State { Idle, FwdSettle, FwdCenter, Coast, Brake, Reverse };

struct Spec {
  bool     enabled;
  bool     allow_repeat;
  uint8_t  weight;
  uint16_t min_ms;
  uint16_t max_ms;
};

  using FlickerFn = void(*)(uint32_t on_ms, uint32_t off_ms, uint16_t cycles, bool invert, bool interrupt);
  void set_flicker_cb(FlickerFn fn);


// lifecycle
void init();              // set default specs, not running
void start();
void stop();
void pause(bool p);
bool running();
bool paused();

// tick every loop()
void tick();

// knobs
void set_log(bool on);
void set_random(bool on);       bool random_enabled();

void enable(State s, bool en);
void allow_repeat(State s, bool on);
void set_duration(State s, uint16_t min_ms, uint16_t max_ms);
void set_weight(State s, uint8_t w);

void set_run_duty(int v);       int  run_duty();
void set_brake_duty(uint8_t v); uint8_t brake_duty();

// status
State current();
const char* state_name(State s);
void status(Print& out);

} // namespace routine
