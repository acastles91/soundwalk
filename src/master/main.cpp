#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <ctype.h>

#include <Adafruit_DotStar.h>
#include <math.h>

#include <message.h>   // shared message types
#include <motion.h>    // BLDC module
#include <actuator.h>  // DRV8833 module
#include <pins.h>      // pin/channel definitions

// -------------------- Local DotStar (optional visual) --------------------
#define MASTER_NUM_LEDS 30
Adafruit_DotStar strip(MASTER_NUM_LEDS, MASTER_DATAPIN, MASTER_CLOCKPIN, DOTSTAR_BRG);

// -------------------- ESP-NOW peers --------------------
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

// -------------------- ESP-NOW helpers --------------------
static uint32_t g_seq = 1;
static BreathMsg  last_breath{};
static bool       have_last_breath = false;
static FlickerMsg last_flicker{};
static bool       have_last_flicker = false;

static void addPeer(const uint8_t mac[6]) {
  esp_now_peer_info_t p{};
  memcpy(p.peer_addr, mac, 6);
  p.channel = 0;
  p.encrypt = false;
  esp_now_add_peer(&p);
}
static void onDataSent(const uint8_t*, esp_now_send_status_t s) {
  Serial.println(s == ESP_NOW_SEND_SUCCESS ? "ESP-NOW send ok" : "ESP-NOW send FAIL");
}
static void setupESPNow() {
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init error");
    return;
  }
  esp_now_register_send_cb(onDataSent);
  if (NUM_SLAVES > 0) addPeer(PEERS[0]); // first only; slaves forward
}
static inline void send_to_first_slave(const void* data, size_t len) {
  if (NUM_SLAVES == 0) return;
  esp_err_t err = esp_now_send(PEERS[0], (const uint8_t*)data, len);
  if (err != ESP_OK) Serial.printf("esp_now_send err=%d\n", err);
}

// -------------------- Commands sent by master --------------------
static void startBreathAll(uint8_t r,uint8_t g,uint8_t b,
                           float bmin,float bmax,
                           uint32_t up_ms,uint32_t down_ms,
                           uint16_t cycles,
                           bool interrupt=false,
                           uint8_t ttl=40,
                           uint32_t start_offset=500)
{
  BreathMsg m{};
  m.mode  = MODE_BREATH;
  m.r = r; m.g = g; m.b = b;
  m.b_min = bmin < 0 ? 0 : (bmin > 1 ? 1 : bmin);
  m.b_max = bmax < 0 ? 0 : (bmax > 1 ? 1 : bmax);
  if (m.b_max < m.b_min) { float t = m.b_min; m.b_min = m.b_max; m.b_max = t; }
  m.up_ms   = up_ms   ? up_ms   : 1;
  m.down_ms = down_ms ? down_ms : 1;
  m.cycles  = cycles;
  m.seq     = g_seq++;
  m.flags   = interrupt ? F_INTERRUPT : 0;
  m.ttl     = ttl;
  m.t0_ms   = millis() + start_offset;

  send_to_first_slave(&m, sizeof(m));
  last_breath = m;
  have_last_breath = true;
  Serial.println("BREATH: command sent");
}

static void startFlickerAll(uint32_t on_ms,
                            uint32_t off_ms,
                            uint16_t cycles,
                            bool invert=false,
                            bool interrupt=false,
                            uint8_t ttl=40,
                            uint32_t start_offset=300)
{
  FlickerMsg f{};
  f.mode   = MODE_FLICKER;
  f.on_ms  = on_ms  ? on_ms  : 1;
  f.off_ms = off_ms ? off_ms : 1;
  f.cycles = cycles;
  f.invert = invert ? 1 : 0;
  f.seq    = g_seq++;
  f.flags  = interrupt ? F_INTERRUPT : 0;
  f.ttl    = ttl;
  f.t0_ms  = millis() + start_offset;

  send_to_first_slave(&f, sizeof(f));
  last_flicker = f;
  have_last_flicker = true;
  Serial.println("FLICKER: command sent");
}

static void startTestChain(uint16_t step_ms,
                           uint8_t r, uint8_t g, uint8_t b,
                           uint8_t ttl = 60,
                           uint32_t start_offset = 500)
{
  TestMsg t{};
  t.mode    = MODE_TEST;
  t.seq     = g_seq++;
  t.flags   = F_INTERRUPT;     // test preempts other effects
  t.ttl     = ttl;
  t.t0_ms   = millis() + start_offset;
  t.step_ms = step_ms;
  t.r = r; t.g = g; t.b = b;
  send_to_first_slave(&t, sizeof(t));
  Serial.println("TEST chain kicked off.");
}

// -------------------- Local DotStar helpers --------------------
static void fillStrip(uint8_t r,uint8_t g,uint8_t b){
  for (int i=0;i<MASTER_NUM_LEDS;i++) strip.setPixelColor(i, r,g,b);
  strip.show();
}

// --- BLDC bring-up helpers (manual sweep) ---
static const int8_t HIN_TAB[6][3] = {
  {1,0,0},{1,0,0},{0,1,0},{0,1,0},{0,0,1},{0,0,1}
};
static const int8_t LIN_TAB[6][3] = {
  {0,1,0},{0,0,1},{0,0,1},{1,0,0},{1,0,0},{0,1,0}
};
static inline void setHIN(uint8_t phase, bool on) {
  uint8_t pin = (phase==0)?HIN1:(phase==1)?HIN2:HIN3;
  pinMode(pin, OUTPUT);
  digitalWrite(pin, on ? HIGH : LOW);
}
static inline void setLIN(uint8_t phase, bool on) {
  uint8_t pin = (phase==0)?LIN1:(phase==1)?LIN2:LIN3;
  pinMode(pin, OUTPUT);
  digitalWrite(pin, on ? HIGH : LOW);
}
static void alignHold(int step = 0, int ms = 600) {
  pinMode(ENABLE, OUTPUT);
  digitalWrite(ENABLE, HIGH);
  for (int i=0;i<3;i++) {
    setHIN(i, HIN_TAB[step][i]);
    setLIN(i, LIN_TAB[step][i]);
  }
  delay(ms);
}
static void commutateSweep(int delay_ms, int cycles) {
  int step = 0;
  for (int c=0; c<cycles; ++c) {
    for (int k=0; k<6; ++k) {
      for (int i=0;i<3;i++) {
        setHIN(i, HIN_TAB[step][i]);
        setLIN(i, LIN_TAB[step][i]);
      }
      delay(delay_ms);
      step = (step+1) % 6;
    }
  }
}
static void motorBringUpOnce() {
  pinMode(HIN1, OUTPUT); pinMode(HIN2, OUTPUT); pinMode(HIN3, OUTPUT);
  pinMode(LIN1, OUTPUT); pinMode(LIN2, OUTPUT); pinMode(LIN3, OUTPUT);
  pinMode(ENABLE, OUTPUT); digitalWrite(ENABLE, HIGH);
  alignHold(0, 800);
  commutateSweep(/*delay_ms*/ 30, /*cycles*/ 20);
  for (int i=0;i<3;i++){ setHIN(i,false); setLIN(i,false); }
}

// -------------------- Console (line-based; echoes each byte) --------------------
static String g_line;

static void printHelp(){
  Serial.println(F(
    "\nActuator console (type then Enter):\n"
    "  f N          -> forward with duty N (0..255)\n"
    "  r N          -> reverse with duty N (0..255)\n"
    "  c            -> coast\n"
    "  b [N]        -> brake, optional duty N (default 255)\n"
    "  burst P T    -> burst with signed power P (-255..255) for T ms\n"
    "  auto on/off  -> enable/disable auto state machine\n"
    "  auto duty N  -> set runDuty for AUTO\n"
    "  auto set f a b  (FORWARD ms range)\n"
    "  auto set r a b  (REVERSE ms range)\n"
    "  auto set k a b  (BRAKE   ms range)\n"
    "  auto set s a b  (COAST   ms range)\n"
    "  brake duty N -> set brakeDuty for AUTO\n"
    "  status       -> print status\n"
    "\nLED / chain (also line-based):\n"
    "  led r | led g | led b\n"
    "  led f on off [cycles]\n"
    "  led c\n"
    "  led t [step_ms] [r g b]\n"
    "  mbringup      (manual BLDC 6-step sweep)\n"
    "  help or ?\n"
  ));
}

static void splitTokens(const String& s, String out[], int& n, int maxN){
  n = 0; int i=0;
  while (i < (int)s.length()){
    while (i<(int)s.length() && isspace((unsigned char)s[i])) i++;
    if (i >= (int)s.length()) break;
    int j=i;
    while (j<(int)s.length() && !isspace((unsigned char)s[j])) j++;
    if (n < maxN) out[n++] = s.substring(i,j);
    i = j;
  }
}
static long toLong(const String& s, long def=0){
  char* e=nullptr; long v=strtol(s.c_str(), &e, 10);
  return (e && *e==0) ? v : def;
}

static void handleActuatorCommand(const String& line){
  String t[8]; int n=0; splitTokens(line, t, n, 8);
  if (n==0) return;

  if (t[0] == "?" || t[0] == "help"){ printHelp(); return; }

  if (t[0] == "f" && n>=2){
    actuator::enableAuto(false);
    actuator::drive(constrain((int)toLong(t[1], 120), 0, 255));
    Serial.printf("MANUAL forward %ld\n", toLong(t[1], 120));
    return;
  }
  if (t[0] == "r" && n>=2){
    actuator::enableAuto(false);
    actuator::drive(-constrain((int)toLong(t[1], 120), 0, 255));
    Serial.printf("MANUAL reverse %ld\n", toLong(t[1], 120));
    return;
  }
  if (t[0] == "c"){
    actuator::enableAuto(false);
    actuator::coast();
    Serial.println("COAST");
    return;
  }
  if (t[0] == "b"){
    actuator::enableAuto(false);
    int d = (n>=2) ? constrain((int)toLong(t[1],255),0,255) : 255;
    actuator::brake(d);
    Serial.printf("BRAKE duty=%d\n", d);
    return;
  }
  if (t[0] == "burst" && n>=3){
    actuator::enableAuto(false);
    int p  = constrain((int)toLong(t[1], 200), -255, 255);
    int ms = max(0,(int)toLong(t[2], 100));
    actuator::burst(p, (uint16_t)ms);
    Serial.printf("BURST power=%d ms=%d\n", p, ms);
    return;
  }
  if (t[0] == "auto" && n>=2){
    if (t[1] == "on"){ actuator::enableAuto(true);  Serial.println("AUTO on");  return; }
    if (t[1] == "off"){ actuator::enableAuto(false); Serial.println("AUTO off"); return; }
    if (t[1] == "duty" && n>=3){
      auto cfg = actuator::getAutoConfig();
      cfg.runDuty = constrain((int)toLong(t[2], cfg.runDuty), 0, 255);
      actuator::setAutoConfig(cfg);
      Serial.printf("AUTO runDuty=%d\n", cfg.runDuty);
      return;
    }
    if (t[1] == "set" && n>=5){
      auto cfg = actuator::getAutoConfig();
      uint16_t a = (uint16_t)max(0L, toLong(t[3],0));
      uint16_t b = (uint16_t)max(0L, toLong(t[4],0));
      if      (t[2]=="f"){ cfg.fwdMinMs=a;   cfg.fwdMaxMs=b; }
      else if (t[2]=="r"){ cfg.revMinMs=a;   cfg.revMaxMs=b; }
      else if (t[2]=="k"){ cfg.brakeMinMs=a; cfg.brakeMaxMs=b; }
      else if (t[2]=="s"){ cfg.coastMinMs=a; cfg.coastMaxMs=b; }
      actuator::setAutoConfig(cfg);
      Serial.printf("AUTO set %s=(%u..%u)ms\n", t[2].c_str(), a,b);
      return;
    }
  }
  if (t[0] == "brake" && n>=3 && t[1] == "duty"){
    auto cfg = actuator::getAutoConfig();
    cfg.brakeDuty = constrain((int)toLong(t[2], cfg.brakeDuty), 0, 255);
    actuator::setAutoConfig(cfg);
    Serial.printf("AUTO brakeDuty=%d\n", cfg.brakeDuty);
    return;
  }
  if (t[0] == "status"){ actuator::debugPrint(Serial); return; }

  // LED / chain commands (also line-based)
  if (t[0] == "led"){
    if (n<2){ printHelp(); return; }
    const String& sub = t[1];
    if (sub=="r"){ startBreathAll(255,0,0, 0.05f,0.6f, 900,1100, 0, true); Serial.println("BREATH: red");   return; }
    if (sub=="g"){ startBreathAll(0,255,0, 0.05f,0.6f, 900,1100, 0, true); Serial.println("BREATH: green"); return; }
    if (sub=="b"){ startBreathAll(0,0,255, 0.05f,0.7f, 1000,1200, 0, true);Serial.println("BREATH: blue");  return; }
    if (sub=="c"){ startFlickerAll(1,0,1, false, true);                    Serial.println("FLICKER: cleared"); return; }
    if (sub=="f"){
      uint32_t on  = (n>=3)? (uint32_t)toLong(t[2],20) : 20;
      uint32_t off = (n>=4)? (uint32_t)toLong(t[3],20) : 20;
      uint16_t cyc = (n>=5)? (uint16_t)toLong(t[4],40) : 40;
      startFlickerAll(on, off, cyc, false, true);
      Serial.printf("FLICKER: %u/%u x%u\n", (unsigned)on,(unsigned)off,(unsigned)cyc);
      return;
    }
    if (sub=="t"){
      uint16_t step = (n>=3)? (uint16_t)toLong(t[2],50) : 50;
      uint8_t  rr   = (n>=4)? (uint8_t)toLong(t[3],255):255;
      uint8_t  gg   = (n>=5)? (uint8_t)toLong(t[4],0)  :0;
      uint8_t  bb   = (n>=6)? (uint8_t)toLong(t[5],0)  :0;
      startTestChain(step, rr, gg, bb, 60);
      Serial.printf("TEST: step=%u color=(%u,%u,%u)\n", step, rr,gg,bb);
      return;
    }
    printHelp(); return;
  }

  if (t[0] == "mbringup"){ Serial.println("Manual 6-step bring-up…"); motorBringUpOnce(); return; }

  // convenience: accept single-letter lines as commands (still line-based)
  if (t[0].length()==1 && n==1){
    char k = t[0][0];
    if (k=='b'){ startBreathAll(0,0,255, 0.05f,0.7f, 1000,1200, 0, true); Serial.println("BREATH: blue");  return; }
    if (k=='r'){ startBreathAll(255,0,0, 0.05f,0.6f,  900,1100, 0, true); Serial.println("BREATH: red");   return; }
    if (k=='g'){ startBreathAll(0,255,0, 0.05f,0.6f,  900,1100, 0, true); Serial.println("BREATH: green"); return; }
    if (k=='f'){ startFlickerAll(20,20, 40, false, true);                 Serial.println("FLICKER: 20/20 x40"); return; }
    if (k=='c'){ startFlickerAll(1,0,1, false, true);                      Serial.println("FLICKER: cleared");    return; }
    if (k=='t'){ startTestChain(50, 255,0,0, 60);                           Serial.println("TEST chain kicked off."); return; }
    if (k=='x'){ motion::startOpenLoop();                                   Serial.println("MOTOR: start open-loop"); return; }
    if (k=='X'){ motion::stop();                                            Serial.println("MOTOR: stopped");        return; }
  }

  Serial.println("Unknown command. Type '?' for help.");
}

static void handleConsoleInput(){
  while (Serial.available()){
    int ch = Serial.read();
    if (ch < 0) break;

    // echo each byte like your preferred logs
    Serial.printf("[key] 0x%02X '%c'\n", ch, (ch >= 32 && ch <= 126) ? ch : '.');

    if (ch == '\r') continue;
    if (ch == '\n'){
      String line = g_line; g_line = "";
      if (line.length()>0) handleActuatorCommand(line);
    } else {
      g_line += (char)ch;
    }
  }
}

// -------------------- Arduino setup/loop --------------------
void setup() {
  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);

  strip.begin();
  strip.show();

  setupESPNow();
  Serial.print("Master MAC: "); Serial.println(WiFi.macAddress());

  // --- MOTOR (BLDC) ---
  motion::Config mcfg;
  mcfg.min_delay_us   = 12 * 1000;
  mcfg.max_delay_us   = 55 * 1000;
  mcfg.ramp_k         = 0.65f;
  mcfg.base_pwm_duty  = PWM_DEFAULT_DUTY;
  mcfg.use_zero_cross = false;
  motion::setup(mcfg);
  motion::startOpenLoop();

  // --- ACTUATOR (DRV8833) ---
  actuator::setup(AIN1, AIN2, AIN1_CH, AIN2_CH, /*freq*/40, /*res*/8);
  actuator::AutoConfig acfg;
  actuator::setAutoConfig(acfg);
  actuator::enableAuto(true);

  // Default breath: visual confirmation after boot
  startBreathAll(0,0,255, 0.05f,0.6f, 1200,1400, 0, true);

  fillStrip(0,0,40);
  printHelp();
}

void loop() {
  const uint32_t now = millis();

  handleConsoleInput();   // line-based console with per-byte echo

  motion::tick();
  actuator::tick();

  static uint32_t led_ms = 0;
  if (now - led_ms > 500) {
    led_ms = now;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}


