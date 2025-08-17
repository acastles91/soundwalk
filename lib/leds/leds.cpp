#include "leds.h"
#include <Arduino.h>
#include <math.h>

namespace leds {

static Adafruit_DotStar* s_strip = nullptr;
static uint16_t s_count = 0;
static uint8_t s_spacing = 1;     // group size
static uint8_t s_onCount = 1;     // how many ON in each group
static uint8_t s_defR = 0, s_defG = 0, s_defB = 127;

void setup(uint16_t count, uint8_t dataPin, uint8_t clockPin, uint8_t order) {
  if (s_strip) { delete s_strip; s_strip = nullptr; }
  s_count  = count;
  s_strip  = new Adafruit_DotStar(count, dataPin, clockPin, order);
  s_strip->begin();
  s_strip->clear();
  s_strip->show();
}

void setBrightness(uint8_t b){ if (s_strip) s_strip->setBrightness(b); }

void setGrouping(uint8_t spacing, uint8_t onCount){
  s_spacing = spacing ? spacing : 1;
  s_onCount = onCount ? onCount : 1;
  if (s_onCount > s_spacing) s_onCount = s_spacing;
}

void setDefaultFlickerColor(uint8_t r,uint8_t g,uint8_t b){ s_defR=r; s_defG=g; s_defB=b; }

void clear(){
  if (!s_strip) return;
  s_strip->clear();
  s_strip->show();
}

void fill(uint8_t r,uint8_t g,uint8_t b){
  if (!s_strip) return;
  for (int i=0;i<s_count;++i){
    if ((i % s_spacing) < s_onCount) s_strip->setPixelColor(i, r, g, b);
    else                             s_strip->setPixelColor(i, 0, 0, 0);
  }
  s_strip->show();
}

void show(){ if (s_strip) s_strip->show(); }

void selfTest(){
  if (!s_strip) return;
  fill(255,0,0); delay(120);
  fill(0,255,0); delay(120);
  fill(0,0,255); delay(120);
  fill(0,0,0);
}

// ---------- math ----------
float clamp01(float x){ return x<0?0:(x>1?1:x); }
float easeCos(float x){ return 0.5f * (1.0f - cosf(3.1415926f * x)); }

float breathBrightness(uint32_t now, const BreathMsg& p){
  if (p.mode != MODE_BREATH) return 0.0f;
  if (now < p.t0_ms) return p.b_min;

  const uint32_t period = p.up_ms + p.down_ms;
  if (!period) return p.b_min;

  const uint32_t elapsed = now - p.t0_ms;
  if (p.cycles){
    const uint32_t total = (uint32_t)p.cycles * period;
    if (elapsed >= total) return p.b_min;
  }

  const uint32_t t = elapsed % period;
  float a = (t < p.up_ms)
            ? easeCos((float)t / (float)p.up_ms)
            : easeCos(1.0f - (float)(t - p.up_ms) / (float)p.down_ms);

  return p.b_min + (p.b_max - p.b_min) * a;
}

bool breathFinished(uint32_t now, const BreathMsg& p){
  if (p.mode != MODE_BREATH) return true;
  const uint32_t period = p.up_ms + p.down_ms;
  if (!period) return true;
  if (p.cycles == 0) return false;
  if (now < p.t0_ms) return false;
  const uint32_t total = (uint32_t)p.cycles * period;
  return (now - p.t0_ms) >= total;
}

uint8_t flickerGate(uint32_t now, const FlickerMsg& f){
  if (f.mode != MODE_FLICKER) return 1;
  if (now < f.t0_ms) return 0;

  const uint32_t period = (uint32_t)f.on_ms + f.off_ms;
  if (!period) return 1;

  if (f.cycles){
    const uint32_t total = (uint32_t)f.cycles * period;
    if ((now - f.t0_ms) >= total) return 1;  // finished => open gate
  }

  const uint32_t t = (now - f.t0_ms) % period;
  uint8_t g = (t < f.on_ms) ? 1 : 0;
  return f.invert ? !g : g;
}

// ---------- render ----------
void renderBreath(uint32_t now, const BreathMsg& p){
  if (!s_strip) return;
  const float b = clamp01(breathBrightness(now, p));
  const uint8_t rr = (uint8_t)(p.r * b);
  const uint8_t gg = (uint8_t)(p.g * b);
  const uint8_t bb = (uint8_t)(p.b * b);

  for (int i=0;i<s_count;++i){
    if ((i % s_spacing) < s_onCount) s_strip->setPixelColor(i, rr, gg, bb);
    else                             s_strip->setPixelColor(i, 0, 0, 0);
  }
  s_strip->show();
}

void renderFlickerOnly(uint32_t now, const FlickerMsg& f){
  if (!s_strip) return;
  const uint8_t gate = flickerGate(now, f);
  const uint8_t rr = s_defR * gate;
  const uint8_t gg = s_defG * gate;
  const uint8_t bb = s_defB * gate;

  for (int i=0;i<s_count;++i){
    if ((i % s_spacing) < s_onCount) s_strip->setPixelColor(i, rr, gg, bb);
    else                             s_strip->setPixelColor(i, 0, 0, 0);
  }
  s_strip->show();
}

void renderCombined(uint32_t now, const BreathMsg& breath, const FlickerMsg& flicker){
  if (!s_strip) return;

  if (breath.mode != MODE_BREATH && flicker.mode == MODE_FLICKER){
    renderFlickerOnly(now, flicker);
    return;
  }

  const float breathVal = clamp01(breathBrightness(now, breath));
  const uint8_t gate    = flickerGate(now, flicker);
  const float k         = breathVal * (gate ? 1.0f : 0.0f);

  const uint8_t rr = (uint8_t)(breath.r * k);
  const uint8_t gg = (uint8_t)(breath.g * k);
  const uint8_t bb = (uint8_t)(breath.b * k);

  for (int i=0;i<s_count;++i){
    if ((i % s_spacing) < s_onCount) s_strip->setPixelColor(i, rr, gg, bb);
    else                             s_strip->setPixelColor(i, 0, 0, 0);
  }
  s_strip->show();
}

void setPixel(uint16_t i, uint8_t r, uint8_t g, uint8_t b) {   // <-- implement this
  if (!s_strip) return;
  if (i >= s_count) return;
  s_strip->setPixelColor(i, r, g, b);
}

} // namespace leds
