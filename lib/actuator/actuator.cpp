#include "actuator.h"

namespace actuator {

static int s_pinA=-1, s_pinB=-1;
static int s_chA=-1,  s_chB=-1;
static int s_pwm_hz=40, s_pwm_res_bits=8;

static volatile Mode    s_mode = Mode::IDLE;
static volatile int16_t s_power = 0;        // -255..255 (effective output when MANUAL/AUTO running)
static uint8_t          s_manualDuty = 120; // default for MANUAL if you call setManualDuty()

// Burst
static bool      s_burstActive = false;
static uint32_t  s_burstEndMs  = 0;
static int16_t   s_burstPower  = 0;

// Auto SM
static bool       s_autoEnabled = false;
static AutoConfig s_cfg;
static uint32_t   s_stateEndMs = 0;
static Mode       s_autoState  = Mode::COAST;

// Helpers
static inline uint8_t clamp8(int v){ if (v<0) v=0; if (v>255) v=255; return (uint8_t)v; }
static inline uint16_t rrange(uint16_t a, uint16_t b){
  if (a>b) { uint16_t t=a; a=b; b=t; }
  if (a==b) return a;
  return a + (uint16_t)random((long)(b-a+1));
}

static void writeDual(uint8_t da, uint8_t db){
  ledcWrite(s_chA, da);
  ledcWrite(s_chB, db);
}

static void applySignedPower(int16_t p){
  // p>0 => forward on pinA; p<0 => reverse on pinB
  if (p > 0) {
    writeDual(clamp8(p), 0);
  } else if (p < 0) {
    writeDual(0, clamp8(-p));
  } else {
    // coast
    writeDual(0, 0);
  }
}

void setup(int pinA, int pinB, int chA, int chB, int pwm_hz, int pwm_res_bits){
  s_pinA = pinA; s_pinB = pinB; s_chA = chA; s_chB = chB;
  s_pwm_hz = pwm_hz; s_pwm_res_bits = pwm_res_bits;

  pinMode(s_pinA, OUTPUT);
  pinMode(s_pinB, OUTPUT);

  ledcSetup(s_chA, s_pwm_hz, s_pwm_res_bits);
  ledcAttachPin(s_pinA, s_chA);
  ledcSetup(s_chB, s_pwm_hz, s_pwm_res_bits);
  ledcAttachPin(s_pinB, s_chB);

  writeDual(0,0);
  s_mode = Mode::IDLE;
  s_power = 0;
}

void setManualDuty(uint8_t duty){ s_manualDuty = duty; }

void drive(int16_t power){
  s_autoEnabled = false;
  s_burstActive = false;
  s_mode = Mode::MANUAL;
  s_power = constrain(power, -255, 255);
  applySignedPower(s_power);
}

void coast(){
  s_autoEnabled = false;
  s_burstActive = false;
  s_mode = Mode::COAST;
  s_power = 0;
  writeDual(0,0);
}

void brake(uint8_t duty){
  s_autoEnabled = false;
  s_burstActive = false;
  s_mode = Mode::BRAKE;
  s_power = 0;
  duty = clamp8(duty);
  // simple dynamic brake: both high with PWM
  writeDual(duty, duty);
}

void stop(){ coast(); }

void burst(int16_t power, uint16_t ms){
  s_burstActive = true;
  s_burstPower  = constrain(power, -255, 255);
  s_burstEndMs  = millis() + ms;
  // immediate apply
  applySignedPower(s_burstPower);
}

void enableAuto(bool on){
  s_autoEnabled = on;
  if (on){
    s_mode = Mode::AUTO;
    // start with coast a bit
    s_autoState = Mode::COAST;
    s_stateEndMs = millis() + rrange(s_cfg.coastMinMs, s_cfg.coastMaxMs);
    writeDual(0,0);
  } else {
    // leave outputs as-is; next manual call will set them
  }
}

bool isAuto(){ return s_autoEnabled; }

void setAutoConfig(const AutoConfig& cfg){ s_cfg = cfg; }
AutoConfig getAutoConfig(){ return s_cfg; }

Mode mode(){ return s_mode; }
int16_t currentPower(){ return s_power; }

void tick(){
  const uint32_t now = millis();

  // Handle burst override
  if (s_burstActive){
    if ( (int32_t)(now - s_burstEndMs) >= 0 ){
      s_burstActive = false;
      // resume whatever mode we were in
      if (s_mode == Mode::MANUAL){
        applySignedPower(s_power);
      } else if (s_mode == Mode::BRAKE){
        writeDual(s_cfg.brakeDuty, s_cfg.brakeDuty);
      } else if (s_mode == Mode::COAST || s_mode == Mode::IDLE){
        writeDual(0,0);
      }
    } else {
      // still bursting; keep forced power
      applySignedPower(s_burstPower);
      return;
    }
  }

  // AUTO state machine
  if (s_autoEnabled && s_mode == Mode::AUTO){
    if ((int32_t)(now - s_stateEndMs) >= 0){
      // choose next state and duration
      // options: FORWARD, REVERSE, COAST, BRAKE
      Mode next = Mode::FORWARD;
      if (s_cfg.randomize){
        const uint8_t pick = (uint8_t)random(4);
        next = (pick==0)?Mode::FORWARD : (pick==1)?Mode::REVERSE : (pick==2)?Mode::COAST : Mode::BRAKE;
      } else {
        // simple cycle
        next = (s_autoState==Mode::FORWARD)?Mode::REVERSE :
               (s_autoState==Mode::REVERSE)?Mode::COAST :
               (s_autoState==Mode::COAST)?Mode::BRAKE : Mode::FORWARD;
      }

      s_autoState = next;
      switch (next){
        case Mode::FORWARD:
          s_stateEndMs = now + rrange(s_cfg.fwdMinMs, s_cfg.fwdMaxMs);
          s_power = s_cfg.runDuty;
          applySignedPower(s_power);
          break;
        case Mode::REVERSE:
          s_stateEndMs = now + rrange(s_cfg.revMinMs, s_cfg.revMaxMs);
          s_power = -((int16_t)s_cfg.runDuty);
          applySignedPower(s_power);
          break;
        case Mode::COAST:
          s_stateEndMs = now + rrange(s_cfg.coastMinMs, s_cfg.coastMaxMs);
          s_power = 0;
          writeDual(0,0);
          break;
        case Mode::BRAKE:
          s_stateEndMs = now + rrange(s_cfg.brakeMinMs, s_cfg.brakeMaxMs);
          s_power = 0;
          writeDual(s_cfg.brakeDuty, s_cfg.brakeDuty);
          break;
        default: break;
      }
    }
    return;
  }

  // MANUAL keeps whatever you last set
  if (s_mode == Mode::MANUAL){
    applySignedPower(s_power);
    return;
  }

  // BRAKE/COAST/IDLE: re-assert outputs
  if (s_mode == Mode::BRAKE){
    writeDual(s_cfg.brakeDuty, s_cfg.brakeDuty);
  } else {
    writeDual(0,0);
  }
}

void debugPrint(Stream& s){
  s.print("[act] mode=");
  switch (s_mode){
    case Mode::IDLE: s.print("IDLE"); break;
    case Mode::MANUAL: s.print("MANUAL"); break;
    case Mode::FORWARD: s.print("FWD"); break;
    case Mode::REVERSE: s.print("REV"); break;
    case Mode::COAST: s.print("COAST"); break;
    case Mode::BRAKE: s.print("BRAKE"); break;
    case Mode::AUTO: s.print("AUTO"); break;
  }
  s.print(" power="); s.print((int)s_power);
  s.print(" burst="); s.print(s_burstActive ? "ON" : "off");
  s.print(" auto="); s.println(s_autoEnabled ? "ON" : "off");
}

} // namespace actuator
