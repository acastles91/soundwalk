#pragma once
#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <stdint.h>
#include <stddef.h>
#include <message.h>   // lives in <project>/include

namespace comms {
namespace espnow {

// App-level callbacks (called from WiFi task context, NOT an ISR)
using breath_cb_t  = void (*)(const uint8_t from[6], const BreathMsg& rebased);
using flicker_cb_t = void (*)(const uint8_t from[6], const FlickerMsg& rebased);
using test_cb_t    = void (*)(const uint8_t from[6], const TestMsg&   rebased);

// Initialize ESP-NOW for a daisy chain.
// - peers: pointer to a [N][6] MAC table (not copied; must remain valid)
// - num_peers: number of peers
// - my_index: index of *this* device in peers (0..N-1)
// - bcb/fcb/tcb: callbacks (nullptr allowed)
void init(const uint8_t (*peers)[6], size_t num_peers, size_t my_index,
          breath_cb_t bcb, flicker_cb_t fcb, test_cb_t tcb);

// Keep link healthy (re-add next peer if it drops)
void tick();

// Send arbitrary payload to peer by index (0..num_peers-1)
bool send_to_index(size_t idx, const void* buf, size_t len);

// Optional: verbose logs
void set_verbose(bool v);

// Info
size_t my_index();
const uint8_t* mac_of(size_t idx);

} // namespace espnow
} // namespace comms
