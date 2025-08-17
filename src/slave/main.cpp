#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <message.h>
#include <leds.h>
#include <espnow.h>

// -------------------- Hardware config --------------------
#define NUM_LEDS    144
#define DATAPIN     5
#define CLOCKPIN    4
#define SLAVE_INDEX 0

// Grouping (e.g., 1 on, 2 off, repeat)
#define LED_SPACING   3
#define LED_ON_COUNT  1

// -------------------- ESP-NOW peers ----------------------
static const uint8_t PEERS[][6] = {
  {0xC0,0x5D,0x89,0xDC,0xA9,0xDC}, // Slave 0
  {0xC0,0x5D,0x89,0xDC,0x94,0x2C}, // Slave 1
  {0xC0,0x5D,0x89,0xDC,0xA9,0xC0}, // Slave 2
  {0xC0,0x5D,0x89,0xDC,0x99,0x7C}, // Slave 3
  {0xC0,0x5D,0x89,0xDC,0x98,0x08}, // Slave 4
  {0x14,0x2B,0x2F,0xDD,0x69,0xA4}, // Slave 5
  {0xC0,0x5D,0x89,0xDC,0x6E,0xC0}, // Slave 6
  {0xC0,0x5D,0x89,0xDC,0x9A,0x9C}, // Slave 7
  {0xC0,0x5D,0x89,0xDC,0x9B,0xD4}, // Slave 8
  {0xC0,0x5D,0x89,0xDC,0x95,0xD0}, // Slave 9
  {0xC0,0x5D,0x89,0xDC,0xA9,0x28}, // Slave 10
  {0xC0,0x5D,0x89,0xDC,0x9B,0x24}, // Slave 11
  {0xC0,0x5D,0x89,0xDC,0x98,0xB4}, // Slave 12
  {0xC0,0x5D,0x89,0xDC,0xA9,0x2C}, // Slave 13
  {0xC0,0x5D,0x89,0xDC,0x6C,0xCC}, // Slave 14
  {0xC0,0x5D,0x89,0xDC,0x93,0x7C}, // Slave 15
  {0xC0,0x5D,0x89,0xDC,0x7F,0x78}, // Slave 16
  {0xC0,0x5D,0x89,0xDD,0x1E,0x74}, // Slave 17
  {0xC0,0x5D,0x89,0xDC,0x92,0xFC}, // Slave 18
  {0x3C,0x8A,0x1F,0x7D,0xA6,0xE4}, // Slave 19
  {0x38,0x18,0x2B,0x8B,0x85,0xB0}, // Slave 20
  {0x68,0x25,0xDD,0xFD,0x53,0xF4}, // Slave 21
  {0x00,0x4B,0x12,0x33,0x68,0x24}, // Slave 22
  {0x38,0x18,0x2B,0x8B,0x82,0x04}, // Slave 23
  {0x00,0x4B,0x12,0x2E,0xEA,0x14}, // Slave 24
  {0x38,0x18,0x2B,0x8A,0x2A,0xF8}, // Slave 25
  {0x38,0x18,0x2B,0x8B,0xD6,0xBC}, // Slave 26
  {0x68,0x25,0xDD,0xF1,0xB2,0x34}, // Slave 27
  {0x68,0x25,0xDD,0xFD,0x18,0x90}, // Slave 28
};
static const size_t NUM_SLAVES = sizeof(PEERS) / sizeof(PEERS[0]);

// -------------------- State ------------------------------
portMUX_TYPE breathMux = portMUX_INITIALIZER_UNLOCKED;

// BREATH/FLICKER
static BreathMsg  currentBreath = { MODE_NONE };
static volatile bool effect_active = false;
static uint32_t    effect_end_ms = 0;
static uint32_t    last_seq = 0;

static BreathMsg   pendingBreath{};
static volatile bool has_pending = false;

static FlickerMsg  currentFlicker{};
static volatile bool flicker_active = false;
static uint32_t    last_flicker_seq = 0;

static FlickerMsg  pendingFlicker{};
static volatile bool has_pending_flicker = false;

// TEST
static TestMsg     currentTest{};
static volatile bool test_active = false;
static uint32_t    last_test_seq = 0;
static uint16_t    test_idx = 0;
static uint32_t    test_next_ms = 0;

// -------------------- Callbacks from comms::espnow -------
static void on_breath_cb(const uint8_t from[6], const BreathMsg& msg){
  if (effect_active && !(msg.flags & F_INTERRUPT)) return;
  if (msg.seq <= last_seq) return;
  last_seq = msg.seq;

  taskENTER_CRITICAL(&breathMux);
  pendingBreath = msg;
  has_pending   = true;
  taskEXIT_CRITICAL(&breathMux);
}

static void on_flicker_cb(const uint8_t from[6], const FlickerMsg& msg){
  if (flicker_active && !(msg.flags & F_INTERRUPT)) return;
  if (msg.seq <= last_flicker_seq) return;
  last_flicker_seq = msg.seq;

  taskENTER_CRITICAL(&breathMux);
  pendingFlicker       = msg;
  has_pending_flicker  = true;
  taskEXIT_CRITICAL(&breathMux);
}

static void on_test_cb(const uint8_t from[6], const TestMsg& msg){
  // Preempt other effects unless told otherwise
  if (test_active && !(msg.flags & F_INTERRUPT)) return;
  if (msg.seq <= last_test_seq) return;
  last_test_seq = msg.seq;

  currentTest  = msg;
  test_idx     = 0;
  test_next_ms = msg.t0_ms;  // already rebased by comms layer
  test_active  = true;

  // Turn off other effects while testing
  effect_active  = false;
  flicker_active = false;

  Serial.printf("TEST start: seq=%lu step=%u ms color=(%u,%u,%u) ttl=%u\n",
                (unsigned long)msg.seq, (unsigned)msg.step_ms,
                msg.r, msg.g, msg.b, msg.ttl);
}

// -------------------- Setup/Loop -------------------------
void setup() {
  Serial.begin(115200);

  Serial.begin(115200);
  Serial.setRxBufferSize(1024);   // larger UART RX buffer
  Serial.setTimeout(0);

  // LEDs
  leds::setup(NUM_LEDS, DATAPIN, CLOCKPIN, DOTSTAR_BRG);
  leds::setBrightness(255);
  leds::setGrouping(LED_SPACING, LED_ON_COUNT);
  leds::setDefaultFlickerColor(DEFAULT_FLICKER_R, DEFAULT_FLICKER_G, DEFAULT_FLICKER_B);
  leds::clear();

  // ESP-NOW
  comms::espnow::init(PEERS, NUM_SLAVES, SLAVE_INDEX, on_breath_cb, on_flicker_cb, on_test_cb);
  // comms::espnow::set_verbose(true); // if you exposed it

  Serial.print("SLAVE_INDEX: "); Serial.println(SLAVE_INDEX);
  Serial.print("MAC Address: ");  Serial.println(WiFi.macAddress());
}

void loop() {
  const uint32_t now = millis();

  // Keep ESP-NOW link healthy (re-add peer if needed, etc.)
  comms::espnow::tick();

  // --- Commit pending BREATH ---
  if (has_pending) {
    taskENTER_CRITICAL(&breathMux);
    currentBreath = pendingBreath;
    has_pending   = false;
    taskEXIT_CRITICAL(&breathMux);

    const uint32_t period = currentBreath.up_ms + currentBreath.down_ms;
    const uint32_t total  = (!period || currentBreath.cycles == 0)
                              ? 0xFFFFFFFFu
                              : (uint32_t)currentBreath.cycles * period;
    effect_end_ms = currentBreath.t0_ms + total;
    effect_active = true;

    Serial.printf("New BREATH cmd: seq=%lu r=%u g=%u b=%u\n",
                  (unsigned long)currentBreath.seq,
                  currentBreath.r, currentBreath.g, currentBreath.b);
  }

  // --- Commit pending FLICKER ---
  if (has_pending_flicker) {
    taskENTER_CRITICAL(&breathMux);
    currentFlicker       = pendingFlicker;
    has_pending_flicker  = false;
    taskEXIT_CRITICAL(&breathMux);

    flicker_active = true;
    Serial.printf("New FLICKER cmd: seq=%lu\n", (unsigned long)currentFlicker.seq);
  }

  // --- TEST state machine (non-blocking) ---
  if (test_active && now >= test_next_ms) {
    if (test_idx == 0) {
      leds::clear(); // start with all off
    }
    if (test_idx < NUM_LEDS) {
      leds::setPixel(test_idx, currentTest.r, currentTest.g, currentTest.b);
      leds::show();                      // leave each LED on
      test_idx++;
      test_next_ms = now + currentTest.step_ms;
    } else {
      // Finished: keep LEDs as-is, then trigger next slave
      test_active = false;
      const size_t next = SLAVE_INDEX + 1;
      if (next < NUM_SLAVES && currentTest.ttl > 0) {
        TestMsg fwd = currentTest;
        fwd.ttl  = currentTest.ttl - 1;
        fwd.t0_ms = millis() + 100;     // small future start
        // reuse same seq so duplicates are ignored naturally
        comms::espnow::send_to_index(next, &fwd, sizeof(fwd));
        Serial.printf("TEST forwarded to slave %u\n", (unsigned)next);
      } else {
        Serial.println("TEST finished (end of chain or TTL=0)");
      }
    }
  }

  // --- Render BREATH/FLICKER if not in TEST ---
  if (!test_active) {
    if (currentBreath.mode == MODE_NONE && !flicker_active) {
      leds::clear();
    } else {
      leds::renderCombined(now, currentBreath, currentFlicker);
    }
  }

  // --- Effect completion bookkeeping ---
  if (flicker_active) {
    const uint32_t period = (uint32_t)currentFlicker.on_ms + (uint32_t)currentFlicker.off_ms;
    if (currentFlicker.cycles && period &&
        (now - currentFlicker.t0_ms) >= (uint32_t)currentFlicker.cycles * period) {
      flicker_active = false;
      Serial.println("FLICKER finished");
    }
  }
  if (effect_active && leds::breathFinished(now, currentBreath)) {
    effect_active = false;
    currentBreath.mode = MODE_NONE;
    Serial.println("BREATH finished");
  }
}
