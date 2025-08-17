// lib/motion/motion.cpp
#include "motion.h"
#include <Arduino.h>
#include "../hw/pins.h"

namespace motion {

namespace {
  // ------- state -------
  hw_timer_t* timer = nullptr;
  portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

  volatile int step_idx = 0;

  unsigned long start_ms = 0;
  int minDelay_us = 12 * 1000;
  int maxDelay_us = 55 * 1000;
  float rampK = 0.65f;

  volatile int currentDelay_us = 3000; // overwritten at start
  volatile bool running = false;
  volatile bool accelerated = false;

  uint8_t baseDuty = PWM_DEFAULT_DUTY;

  // 6-step tables (same as before)
  const int8_t HIN[6][3] = {
    {1,0,0},{1,0,0},{0,1,0},{0,1,0},{0,0,1},{0,0,1}
  };
  const int8_t LIN[6][3] = {
    {0,1,0},{0,0,1},{0,0,1},{1,0,0},{1,0,0},{0,1,0}
  };

  inline void setHIN_PWM(uint8_t phase, int8_t mode) {
    const uint8_t ch = (phase==0)?HIN1_CH:(phase==1)?HIN2_CH:HIN3_CH;
    ledcWrite(ch, mode ? baseDuty : 0);
  }
  inline void setLIN_PWM(uint8_t phase, int8_t mode) {
    const uint8_t ch = (phase==0)?LIN1_CH:(phase==1)?LIN2_CH:LIN3_CH;
    ledcWrite(ch, mode ? baseDuty : 0);
  }

  // ISR: advances commutation step
  void IRAM_ATTR onPwmISR() {
    portENTER_CRITICAL_ISR(&timerMux);
    for (int i=0;i<3;i++){
      setHIN_PWM(i, HIN[step_idx][i]);
      setLIN_PWM(i, LIN[step_idx][i]);
    }
    step_idx = (step_idx + 1) % 6;
    portEXIT_CRITICAL_ISR(&timerMux);
  }

  inline int smoothDelay_us() {
    unsigned long t = millis() - start_ms;
    float df = minDelay_us + (maxDelay_us - minDelay_us) * expf(-rampK * (float)t / 1000.0f);
    return (int)df;
  }
}

// ------ public API ------
void setup(const Config& cfg) {
  minDelay_us = cfg.min_delay_us;
  maxDelay_us = cfg.max_delay_us;
  rampK       = cfg.ramp_k;
  baseDuty    = cfg.base_pwm_duty;

  // Pins
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(HIN1, OUTPUT); pinMode(LIN1, OUTPUT);
  pinMode(HIN2, OUTPUT); pinMode(LIN2, OUTPUT);
  pinMode(HIN3, OUTPUT); pinMode(LIN3, OUTPUT);
  pinMode(ENABLE, OUTPUT);
  digitalWrite(ENABLE, HIGH);

  // LEDC setup
  ledcSetup(HIN1_CH, PWM_FREQ, PWM_RESOLUTION); ledcAttachPin(HIN1, HIN1_CH);
  ledcSetup(HIN2_CH, PWM_FREQ, PWM_RESOLUTION); ledcAttachPin(HIN2, HIN2_CH);
  ledcSetup(HIN3_CH, PWM_FREQ, PWM_RESOLUTION); ledcAttachPin(HIN3, HIN3_CH);
  ledcSetup(LIN1_CH, PWM_FREQ, PWM_RESOLUTION); ledcAttachPin(LIN1, LIN1_CH);
  ledcSetup(LIN2_CH, PWM_FREQ, PWM_RESOLUTION); ledcAttachPin(LIN2, LIN2_CH);
  ledcSetup(LIN3_CH, PWM_FREQ, PWM_RESOLUTION); ledcAttachPin(LIN3, LIN3_CH);

  // Timer (1us tick: clock/80 prescaler)
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onPwmISR, true);
  running = false;
  accelerated = false;
}

void startOpenLoop() {
  start_ms = millis();
  currentDelay_us = maxDelay_us;

  portENTER_CRITICAL(&timerMux);
  timerAlarmWrite(timer, currentDelay_us, true);
  timerAlarmEnable(timer);
  portEXIT_CRITICAL(&timerMux);

  running = true;
  accelerated = false;
}

void stop() {
  if (!timer) return;
  portENTER_CRITICAL(&timerMux);
  timerAlarmDisable(timer);
  portEXIT_CRITICAL(&timerMux);
  running = false;

  // outputs off
  for (int i=0;i<3;i++){
    setHIN_PWM(i, 0);
    setLIN_PWM(i, 0);
  }
}

void tick() {
  if (!running) return;

  int newDelay = smoothDelay_us();
  // "accelerated" threshold kept from your sketch logic
  if (newDelay <= (minDelay_us + 2500)) {
    accelerated = true;
  }

  if (abs(newDelay - currentDelay_us) > 80) {
    currentDelay_us = newDelay;
    portENTER_CRITICAL(&timerMux);
    timerAlarmWrite(timer, currentDelay_us, true);
    portEXIT_CRITICAL(&timerMux);
  }
}

void setDuty(uint8_t duty) {
  baseDuty = duty;
}

bool isRunning() { return running; }
bool isAccelerated() { return accelerated; }
int  currentDelayMicros(){ return currentDelay_us; }
int  commutationStep(){ return step_idx; }

} // namespace motion
