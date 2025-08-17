#pragma once
#include <stdint.h>

enum : uint8_t {
  MODE_NONE    = 0,
  MODE_BREATH  = 1,
  MODE_FLICKER = 2,
  MODE_TEST = 3
};

enum : uint8_t {
  F_NONE       = 0,
  F_INTERRUPT  = 1 << 0,  // force-interrupt running effect
};

// Default fallback color for flicker if no breath is active
#define DEFAULT_FLICKER_R 0
#define DEFAULT_FLICKER_G 0
#define DEFAULT_FLICKER_B 127 

typedef struct __attribute__((packed)) {
  uint8_t  mode;      // MODE_BREATH, etc.
  uint8_t  flags;     // F_INTERRUPT (optional)
  uint16_t version;   // for future protocol bumps
  uint32_t seq;       // command sequence number (monotonic)
  uint8_t  r,g,b;     // base color
  float    b_min;     // 0..1
  float    b_max;     // 0..1
  uint32_t up_ms;
  uint32_t down_ms;
  uint16_t cycles;    // 0 = infinite
  uint32_t t0_ms;     // absolute start time on millis() timebase
  uint8_t  ttl;       // hop budget for daisy-chain (decrement on forward)
} BreathMsg;

typedef struct __attribute__((packed)) {
  uint8_t  mode;        // = MODE_FLICKER
  uint8_t  ttl;
  uint8_t  flags;       // F_INTERRUPT to pre-empt current flicker
  uint8_t  reserved;    // align

  uint32_t seq;         // monotonic from master
  uint32_t t0_ms;       // future start time, for sync

  uint16_t on_ms;       // gate HIGH duration
  uint16_t off_ms;      // gate LOW duration
  uint16_t cycles;      // 0 = continuous
  uint8_t  invert;      // 0 normal, 1 invert gate
} FlickerMsg;

typedef struct __attribute__((packed)) {
  uint8_t  mode;      // = MODE_TEST
  uint8_t  flags;     // F_INTERRUPT optional
  uint16_t version;   // for future bumps (set 1)
  uint32_t seq;       // monotonic from master
  uint8_t  r, g, b;   // color per LED (suggest 255,255,255)
  uint16_t step_ms;   // delay per LED (e.g. 500)
  uint8_t  ttl;       // how many hops left
  uint32_t t0_ms;     // near-future start on master’s millis
} TestMsg;

static_assert(sizeof(BreathMsg)  == 34, "BreathMsg size mismatch (packing/order)");
static_assert(sizeof(FlickerMsg) == 19, "FlickerMsg size mismatch (packing/order)");
static_assert(sizeof(TestMsg)    == 18, "TestMsg size mismatch (packing/order)");
