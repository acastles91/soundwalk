#include <esp_now.h>
#include <WiFi.h>
#include <Adafruit_DotStar.h>
#include <stdlib.h>

#define NUM_LEDS 400
#define DATAPIN 5
#define CLOCKPIN 4
#define SLAVE_INDEX 0


uint8_t slaveAddress_1[] = { 0xA0, 0xB7, 0x65, 0x49, 0xC8, 0x54 };
uint8_t slaveAddress_2[] = { 0xC8, 0xF0, 0x9E, 0x51, 0x97, 0x9C };

//std::array<uint8_t, 6> slaveAddresses = { slaveAddress_1, slaveAddress_2 };

static const uint8_t PEERS[][6] = {
  {0xA0,0xB7,0x65,0x49,0xC8,0x54}, // Slave 0
  {0xC8,0xF0,0x9E,0x51,0x97,0x9C}, // Slave 1
  // { ... },                       // Slave 2
};
static const size_t NUM_SLAVES = sizeof(PEERS)/sizeof(PEERS[0]);

Adafruit_DotStar strip(NUM_LEDS, DATAPIN, CLOCKPIN, DOTSTAR_BRG);

typedef struct __attribute__((packed)) {
  uint8_t r, g, b;
  float   brightness;   // 0.0 .. 1.0
} ledMessage;

static inline float clamp01(float x){ return x < 0 ? 0 : (x > 1 ? 1 : x); }

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Optional: debug
   Serial.printf("Forwarded to %02X:%02X:%02X:%02X:%02X:%02X status=%d\n",
                 mac_addr[0],mac_addr[1],mac_addr[2],mac_addr[3],mac_addr[4],mac_addr[5], status);
}



void onDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {

  if (len == sizeof(ledMessage)) {
    ledMessage msg;
    memcpy(&msg, incomingData, sizeof(ledMessage));

    // Apply brightness scaling
    float bf = clamp01(msg.brightness);
    //float brightnessFactor = (float)(msg.brightness);  // Adjust brightness (0.0 to 1.0)
    uint8_t r = (uint8_t)(msg.r * bf);
    uint8_t g = (uint8_t)(msg.g * bf);
    uint8_t b = (uint8_t)(msg.b * bf);

    for (int i = 0; i < NUM_LEDS; i++) {
      strip.setPixelColor(i, r, g, b);
    }
    Serial.println("Received: " + String(r) + ", " + String(g) + ", " + String(b));
    strip.show();
    const size_t next = SLAVE_INDEX + 1;
    if (next < NUM_SLAVES) {
    esp_err_t res = esp_now_send(PEERS[next], (uint8_t*)&msg, sizeof(msg));
    if (res != ESP_OK) {
       Serial.println("Forward failed");
      }
    }

  }
}

void addPeer(const uint8_t mac[6]) {
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = 0;     // same channel as STA
  peer.encrypt = false;
  esp_now_add_peer(&peer);
}
void setupESPNow() {
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(onDataRecv);
  esp_now_register_send_cb(onDataSent);

  if (SLAVE_INDEX + 1  < NUM_SLAVES) {
    addPeer(PEERS[SLAVE_INDEX + 1]);
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  strip.begin();
  strip.show();

  setupESPNow();

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(onDataRecv);
  Serial.println("SLAVE_INDEX: " + String(SLAVE_INDEX));
  Serial.println("MAC Address: ");  
  Serial.println(WiFi.macAddress());
}

void loop(){
//  for (int i = 0; i < NUM_LEDS; i++) {
//        strip.setPixelColor(i, strip.Color(255, 255, 0));
//      }
//      strip.show();
}