#pragma once
#include <stdint.h>
#include <Adafruit_DotStar.h>
#include <message.h>

namespace leds {

// ----- init / config -----
void setup(uint16_t count, uint8_t dataPin, uint8_t clockPin, uint8_t order = DOTSTAR_BRG);
void setBrightness(uint8_t b);
void setGrouping(uint8_t spacing, uint8_t onCount);
void setDefaultFlickerColor(uint8_t r,uint8_t g,uint8_t b);

// ----- simple helpers -----
void clear();                        // clears and show()
void fill(uint8_t r,uint8_t g,uint8_t b);
void show();
void selfTest();                     // quick RGB flash

// ----- math helpers (pure) -----
float clamp01(float x);
float easeCos(float x);
float breathBrightness(uint32_t now, const BreathMsg& p);
bool  breathFinished(uint32_t now, const BreathMsg& p);
uint8_t flickerGate(uint32_t now, const FlickerMsg& f);

// ----- renderers (write to the internal strip) -----
void renderBreath(uint32_t now, const BreathMsg& p);
void renderFlickerOnly(uint32_t now, const FlickerMsg& f);
void renderCombined(uint32_t now, const BreathMsg& breath, const FlickerMsg& flicker);


void setPixel(uint16_t i, uint8_t r, uint8_t g, uint8_t b);
uint16_t count();  // returns number of pixels passed to setup()

} // namespace leds

