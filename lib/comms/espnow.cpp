#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <message.h>
#include <espnow.h>

namespace comms {
namespace espnow {

// ---------- sanity: struct sizes must match both sides ----------
static_assert(sizeof(BreathMsg)  == 34, "BreathMsg size mismatch");
static_assert(sizeof(FlickerMsg) == 19, "FlickerMsg size mismatch");
static_assert(sizeof(TestMsg)    == 18, "TestMsg size mismatch");

// ---------- module state ----------
static const uint8_t (*s_peers)[6] = nullptr;
static size_t   s_num  = 0;
static size_t   s_idx  = 0;
static bool     s_verbose = true;   // turn ON while debugging

static bool     s_next_added = false;
static uint32_t s_last_try_ms = 0;

// App callbacks
static breath_cb_t  s_breath_cb  = nullptr;
static flicker_cb_t s_flicker_cb = nullptr;
static test_cb_t    s_test_cb    = nullptr;

static inline void vlog(const char* fmt, ...) {
  if (!s_verbose) return;
  va_list ap; va_start(ap, fmt);
  char buf[192];
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  Serial.println(buf);
}

static inline uint32_t rebase_t0(uint32_t remote_t0) {
  uint32_t now = millis();
  int32_t rel = (int32_t)remote_t0 - (int32_t)now;
  if (rel < 5)    rel = 5;
  if (rel > 2000) rel = 50;
  return now + (uint32_t)rel;
}

static void try_add_next_peer() {
  if (s_idx + 1 >= s_num) return;
  esp_now_peer_info_t p{};
  memcpy(p.peer_addr, s_peers[s_idx + 1], 6);
  p.channel = 0;
  p.encrypt = false;
  esp_err_t e = esp_now_add_peer(&p);
  if (e == ESP_OK || e == ESP_ERR_ESPNOW_EXIST) {
    s_next_added = true;
    vlog("[espnow] next peer added");
  } else {
    s_next_added = false;
    vlog("[espnow] add peer failed (%d)", (int)e);
  }
}

// ---------- esp-now callbacks ----------
static void on_send(const uint8_t* mac, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) {
    // if a forward fails, we’ll try to re-add it in tick()
    s_next_added = false;
  }
  vlog("[espnow] TX to %02X:%02X:%02X:%02X:%02X:%02X status=%d",
       mac[0],mac[1],mac[2],mac[3],mac[4],mac[5], (int)status);
}

static void on_recv(const uint8_t* mac, const uint8_t* data, int len) {
  // Always print a one-line summary so you SEE traffic and size
  Serial.printf("[espnow] RX len=%d (Breath=34 Flicker=19 Test=18) from %02X:%02X:%02X:%02X:%02X:%02X\n",
                len, mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);

  if (len <= 0 || data == nullptr) return;

  // decode by the first byte (mode)
  uint8_t mode = data[0];

  switch (mode) {
    case MODE_BREATH: {
      if (len < (int)sizeof(BreathMsg)) { vlog("[espnow] BREATH too short"); return; }

      // 1) Forward original, unrebased, to NEXT peer (if any and ttl>0)
      if (s_idx + 1 < s_num) {
        BreathMsg fwd; memcpy(&fwd, data, sizeof(fwd));
        if (fwd.ttl > 0) {
          fwd.ttl--;
          esp_now_peer_info_t p{}; memcpy(p.peer_addr, s_peers[s_idx + 1], 6);
          p.channel = 0; p.encrypt = false; esp_now_add_peer(&p); // idempotent
          esp_err_t e = esp_now_send(s_peers[s_idx + 1], (const uint8_t*)&fwd, sizeof(fwd));
          if (e != ESP_OK) vlog("[espnow] BREATH forward err=%d", (int)e);
          else             vlog("[espnow] BREATH forwarded -> idx %u (ttl=%u)", (unsigned)(s_idx+1), fwd.ttl);
        }
      }

      // 2) Deliver local, rebased copy to the app
      if (s_breath_cb) {
        BreathMsg m; memcpy(&m, data, sizeof(m));
        m.t0_ms = rebase_t0(m.t0_ms);
        vlog("[espnow] dispatch BREATH seq=%lu", (unsigned long)m.seq);
        s_breath_cb(mac, m);
      } else {
        vlog("[espnow] BREATH callback is NULL");
      }
    } break;

    case MODE_FLICKER: {
      if (len < (int)sizeof(FlickerMsg)) { vlog("[espnow] FLICKER too short"); return; }

      // 1) Forward original to NEXT peer (ttl--)
      if (s_idx + 1 < s_num) {
        FlickerMsg fwd; memcpy(&fwd, data, sizeof(fwd));
        if (fwd.ttl > 0) {
          fwd.ttl--;
          esp_now_peer_info_t p{}; memcpy(p.peer_addr, s_peers[s_idx + 1], 6);
          p.channel = 0; p.encrypt = false; esp_now_add_peer(&p); // idempotent
          esp_err_t e = esp_now_send(s_peers[s_idx + 1], (const uint8_t*)&fwd, sizeof(fwd));
          if (e != ESP_OK) vlog("[espnow] FLICKER forward err=%d", (int)e);
          else             vlog("[espnow] FLICKER forwarded -> idx %u (ttl=%u)", (unsigned)(s_idx+1), fwd.ttl);
        }
      }

      // 2) Local, rebased copy
      if (s_flicker_cb) {
        FlickerMsg m; memcpy(&m, data, sizeof(m));
        m.t0_ms = rebase_t0(m.t0_ms);
        vlog("[espnow] dispatch FLICKER seq=%lu", (unsigned long)m.seq);
        s_flicker_cb(mac, m);
      } else {
        vlog("[espnow] FLICKER callback is NULL");
      }
    } break;

    case MODE_TEST: {
      if (len < (int)sizeof(TestMsg)) { vlog("[espnow] TEST too short"); return; }
      // NOTE: TEST is forwarded by the slave *after* completing its local test.
      if (s_test_cb) {
        TestMsg m; memcpy(&m, data, sizeof(m));
        m.t0_ms = rebase_t0(m.t0_ms);
        vlog("[espnow] dispatch TEST seq=%lu ttl=%u", (unsigned long)m.seq, m.ttl);
        s_test_cb(mac, m);
      } else {
        vlog("[espnow] TEST callback is NULL");
      }
    } break;

    default:
      vlog("[espnow] Unknown mode byte: %u", (unsigned)mode);
      break;
  }
}

// ---------- public API ----------
void init(const uint8_t (*peers)[6], size_t num_peers, size_t my_index,
          breath_cb_t bcb, flicker_cb_t fcb, test_cb_t tcb)
{
  s_peers = peers; s_num = num_peers; s_idx = my_index;
  s_breath_cb  = bcb;
  s_flicker_cb = fcb;
  s_test_cb    = tcb;

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("[espnow] init error");
    return;
  }
  esp_now_register_send_cb(on_send);
  esp_now_register_recv_cb(on_recv);

  try_add_next_peer();

  Serial.printf("[espnow] my idx: %u\n", (unsigned)s_idx);
  Serial.printf("[espnow] my MAC: %s\n", WiFi.macAddress().c_str());
  Serial.printf("[espnow] callbacks: breath=%p flicker=%p test=%p\n",
                (void*)s_breath_cb, (void*)s_flicker_cb, (void*)s_test_cb);
}

void tick() {
  uint32_t now = millis();
  if (!s_next_added && (now - s_last_try_ms) > 2000) {
    s_last_try_ms = now;
    if (s_idx + 1 < s_num) {
      esp_now_del_peer(s_peers[s_idx + 1]); // ignore result
    }
    try_add_next_peer();
  }
}

bool send_to_index(size_t idx, const void* buf, size_t len) {
  if (!s_peers || idx >= s_num) return false;
  // Ensure peer exists (idempotent)
  esp_now_peer_info_t p{};
  memcpy(p.peer_addr, s_peers[idx], 6);
  p.channel = 0;
  p.encrypt = false;
  esp_now_add_peer(&p); // OK if already present
  esp_err_t e = esp_now_send(s_peers[idx], (const uint8_t*)buf, len);
  if (e != ESP_OK) {
    vlog("[espnow] send_to_index %u failed err=%d", (unsigned)idx, (int)e);
  }
  return e == ESP_OK;
}

void set_verbose(bool v) { s_verbose = v; }

size_t my_index() { return s_idx; }

const uint8_t* mac_of(size_t idx) {
  if (!s_peers || idx >= s_num) return nullptr;
  return s_peers[idx];
}

} // namespace espnow
} // namespace comms
