#include "routine.h"
#include "center.h"
#include <actuator.h>

namespace routine {

static FlickerFn s_flicker_cb = nullptr;
void set_flicker_cb(FlickerFn fn){ s_flicker_cb = fn; }

static bool     s_on   = false;
static bool     s_paused = false;
static bool     s_log  = true;
static bool     s_rand = true;

static State    s_state = State::Idle;
static uint32_t s_t0    = 0;
static uint32_t s_dur   = 0;

static int      s_runDuty   = 20;   // reverse strength
static uint8_t  s_brakeDuty = 160;  // brake strength

static Spec s_spec[] = {
/* Idle      */ { false, true,  1, 100, 300 },
/* FwdSettle */ { true,  false, 1,  10,  60 },
/* FwdCenter */ { true,  false, 3, 10, 100 },
/* Coast     */ { true,  true,  1,  50,  2000 },
/* Brake     */ { true,  true,  1,  40,  180 },
/* Reverse   */ { true,  false, 2, 10,  100 },
};

static inline int idx(State s){ return (int)s; }

static uint32_t randIn(uint16_t a, uint16_t b) {
  if (a > b) { uint16_t t=a; a=b; b=t; }
  return (uint32_t)random((long)a, (long)b+1);
}

const char* state_name(State s){
  switch (s){
    case State::Idle:      return "Idle";
    case State::FwdSettle: return "FwdSettle";
    case State::FwdCenter: return "FwdCenter";
    case State::Coast:     return "Coast";
    case State::Brake:     return "Brake";
    case State::Reverse:   return "Reverse";
  }
  return "?";
}

static State pickRandomNext(State cur) {
  int curi = idx(cur), total = 0;
  for (int i=0;i<6;i++){
    if (!s_spec[i].enabled) continue;
    if (i==curi && !s_spec[i].allow_repeat) continue;
    total += s_spec[i].weight;
  }
  if (total <= 0) return cur;
  int r = random(1, total+1), acc=0;
  for (int i=0;i<6;i++){
    if (!s_spec[i].enabled) continue;
    if (i==curi && !s_spec[i].allow_repeat) continue;
    acc += s_spec[i].weight;
    if (r <= acc) return (State)i;
  }
  return cur;
}

static void enter(State s){
  s_state = s;
  s_t0    = millis();
  s_dur   = randIn(s_spec[idx(s)].min_ms, s_spec[idx(s)].max_ms);

  actuator::enableAuto(false);
  center::off();

  switch (s){
    case State::Idle:
    case State::FwdSettle:
      actuator::coast();
      if (s_flicker_cb) s_flicker_cb(1,0,1,false,true); // clear
      break;
    case State::FwdCenter:
      center::on();
      if (s_flicker_cb) s_flicker_cb(20, 20, 40, false, true); // 40 cycles
      break;
    case State::Coast:
      actuator::coast();
      if (s_flicker_cb) s_flicker_cb(1,0,1,false,true); // clear
      break;
    case State::Brake:
      actuator::brake(s_brakeDuty);
      if (s_flicker_cb) s_flicker_cb(1,0,1,false,true); // clear
      break;
    case State::Reverse: {
      int cmd = constrain(s_runDuty, 0, 255);
      actuator::drive(-cmd);
      if (s_flicker_cb) s_flicker_cb(20, 20, 40, false, true); // 40 cycles
      //if (s_log) Serial.printf("  Reverse duty=%d\n", cmd);
      break;
    }

  }

  if (s_log) Serial.printf("ROUTINE -> %-9s dur=%lu ms\n", state_name(s), (unsigned long)s_dur);
}

void init(){ s_on=false; s_paused=false; s_state=State::Idle; s_t0=0; s_dur=0; }

void start(){
  if (!s_on) Serial.println("ROUTINE: on");
  s_on = true; s_paused = false;
  enter(State::FwdSettle);
}

void stop(){
  if (s_on) Serial.println("ROUTINE: off");
  s_on = false; s_paused = false;
  center::off();
  actuator::coast();
  s_state = State::Idle;
}

void pause(bool p){
  s_paused = p;
  Serial.println(p ? "ROUTINE paused" : "ROUTINE resumed");
  if (!p) s_t0 = millis(); // restart timer window
}

bool running(){ return s_on; }
bool paused(){  return s_paused; }

void tick(){
  if (!s_on || s_paused) return;
  uint32_t now = millis();
  if (now - s_t0 >= s_dur) {
    State next = s_rand ? pickRandomNext(s_state) : s_state;
    enter(next);
  }
}

void set_log(bool on){ s_log = on; }
void set_random(bool on){ s_rand = on; }
bool random_enabled(){ return s_rand; }

void enable(State s, bool en){ s_spec[idx(s)].enabled = en; }
void allow_repeat(State s, bool on){ s_spec[idx(s)].allow_repeat = on; }
void set_duration(State s, uint16_t min_ms, uint16_t max_ms){
  s_spec[idx(s)].min_ms = min_ms;
  s_spec[idx(s)].max_ms = (max_ms < min_ms) ? min_ms : max_ms;
}
void set_weight(State s, uint8_t w){ s_spec[idx(s)].weight = w; }

void set_run_duty(int v){ s_runDuty = constrain(v,0,255); }
int  run_duty(){ return s_runDuty; }
void set_brake_duty(uint8_t v){ s_brakeDuty = v; }
uint8_t brake_duty(){ return s_brakeDuty; }

State current(){ return s_state; }

void status(Print& out){
  out.printf("ROUTINE %s random=%s paused=%s state=%s\n",
    s_on?"ON":"OFF", s_rand?"on":"off", s_paused?"yes":"no", state_name(s_state));
  for (int i=0;i<6;i++){
    out.printf("  %-9s en=%d rep=%d w=%u dur=%u..%u\n",
      state_name((State)i), s_spec[i].enabled, s_spec[i].allow_repeat,
      s_spec[i].weight, s_spec[i].min_ms, s_spec[i].max_ms);
  }
}

// In enter(State::Reverse):

// And when entering any non-Reverse state (or specifically on exit of Reverse):

} // namespace routine
