
/* M7.ino — Portenta H7 (M7 core) + Breakout + ST7789 (170x320)
   v3c
   - Re-add fade-out after splash, fade-in for Home.
   - Non-blocking RPC reads (no readStringUntil).
   - Faster time sync: try known time.google.com IPs before DNS (no freeze).
   - HTTP Date fallback kept; short timeouts.
   - Footer: [W ON]/[W OFF] centered, [BL 0–100%] right; left N1LABS chip.
   - Tiny 'x' between M7/M4 (green=linked, #ed8796 if M4 silent >2s).
   - Brightness + time persisted via LittleFS (MBED).
*/

#include <Arduino.h>
#include <Arduino_PortentaBreakout.h>
#include <Arduino_GFX_Library.h>
#include <SPI.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WiFiClient.h>
#include <RPC.h>
#include <LittleFS_Portenta_H7.h>
#include <time.h>

void bootM4(void);

// ====== USER WIFI CREDENTIALS ======
#ifndef WIFI_SSID
  #define WIFI_SSID "Wolv2.4"
#endif
#ifndef WIFI_PASS
  #define WIFI_PASS "Eternal2049$!"
#endif

// ====== Display / pins ======
#define ROTATION       1
#define COLSTART       35
#define ROWSTART       0
#define INVERT_COLORS  0
#define VIB_ACTIVE_LOW 0

const int         PIN_CS    = SPI1_CS;
const int         PIN_DC    = GPIO_2;
const int         PIN_RST   = GPIO_1;
const breakoutPin PIN_BL    = PWM3;     // ACTIVE-LOW PWM (higher value = brighter)
const breakoutPin PIN_VIBR  = PWM4;

const int BTN_RIGHT = GPIO_3;
const int BTN_DOWN  = GPIO_4;
const int BTN_LEFT  = GPIO_5;  // manual M4 reboot + RPC relink
const int BTN_UP    = GPIO_6;
const int BTN_OK    = GPIO_0;  // long-press => manual time sync

// ====== Colors ======
#define C565(r,g,b) ( ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b) >> 3) )
const uint16_t COL_BG       = C565(68, 74, 125);   // #444a7d
const uint16_t COL_TEXT     = C565(235,239,245);
const uint16_t COL_CHIP     = C565(166,218,149);   // #a6da95
const uint16_t COL_CHIPFG   = COL_BG;
const uint16_t COL_TIME_BG  = C565(183,189,248);   // #b7bdf8
const uint16_t COL_DATE_BG  = C565(138,173,244);   // #8aadf4
const uint16_t COL_BADLINK  = C565(237,135,150);   // #ed8796
const uint16_t COL_CLK_TXT  = COL_BG;

const int CHAR_W = 12; // font size 2
const int CHAR_H = 16;
const int VPAD   = 2;

// ====== Storage (LittleFS MBED) ======
LittleFS_MBED *gFS = nullptr;
static const char *PATH_BL   = MBED_LITTLEFS_FILE_PREFIX "/bl.bin";
static const char *PATH_TIME = MBED_LITTLEFS_FILE_PREFIX "/time.bin";

static bool storageBegin(){
  if (gFS) return true;
  gFS = new LittleFS_MBED();
  bool ok = gFS->init();
  Serial.print("[FS] init "); Serial.println(ok ? "OK" : "FAIL");
  return ok;
}
static void saveBrightnessFS(uint8_t v){
  if (!storageBegin()) return;
  FILE *f = fopen(PATH_BL, "wb"); if (!f) return;
  fwrite(&v, 1, 1, f); fclose(f);
}
static bool restoreBrightnessFS(uint8_t &v){
  if (!storageBegin()) return false;
  FILE *f = fopen(PATH_BL, "rb"); if (!f) return false;
  uint8_t t=0; size_t n = fread(&t, 1, 1, f); fclose(f);
  if (n!=1) return false; v=t; return true;
}
static void saveTimeFS(time_t t){
  if (!storageBegin()) return;
  FILE *f = fopen(PATH_TIME, "wb"); if (!f) return;
  int64_t v = (int64_t)t; fwrite(&v, 1, sizeof(v), f); fclose(f);
}
static bool restoreTimeFS(time_t &out){
  if (!storageBegin()) return false;
  FILE *f = fopen(PATH_TIME, "rb"); if (!f) return false;
  int64_t v=0; size_t n = fread(&v, 1, sizeof(v), f); fclose(f);
  if (n != sizeof(v)) return false; out=(time_t)v; return true;
}

// ====== Board Temp (MAX17262 on Wire1) ======
namespace BoardTempInline {
  static const uint8_t ADDR = 0x36;
  static const uint8_t REG_TEMP    = 0x08; // signed, 1/256 °C
  static const uint8_t REG_DIETEMP = 0x34; // signed, 1/256 °C
  static TwoWire &W = Wire1;
  static int       lastC  = -999;
  static uint32_t  lastMs = 0;
  static bool      ready  = false;
  static bool i2c_read16_le(uint8_t reg, uint16_t &out){
    W.beginTransmission(ADDR); W.write(reg);
    if (W.endTransmission(false) != 0) return false;
    if (W.requestFrom((int)ADDR, 2) != 2) return false;
    uint8_t lo = W.read(), hi = W.read();
    out = (uint16_t)((hi<<8)|lo);
    return true;
  }
  static bool readC_from(uint8_t reg, float &outC){
    uint16_t raw; if (!i2c_read16_le(reg, raw)) return false;
    int16_t s = (int16_t)raw; outC = (float)s / 256.0f; return true;
  }
  void begin(){ W.begin(); W.setClock(400000); lastC = -999; lastMs = 0; ready = true; }
  void poll_1Hz(){
    if (!ready) return;
    uint32_t now = millis(); if (now - lastMs < 1000) return; lastMs = now;
    float c = NAN; bool ok = readC_from(REG_TEMP, c); if (!ok) ok = readC_from(REG_DIETEMP, c);
    if (ok && isfinite(c) && c > -40.0f && c < 125.0f) lastC = (int)lroundf(c); else lastC = -999;
  }
  int getC(){ return lastC; }
}
static inline void BoardTemp_begin(){ BoardTempInline::begin(); }
static inline void BoardTemp_poll_1Hz(){ BoardTempInline::poll_1Hz(); }
static inline int  BoardTemp_getC(){ return BoardTempInline::getC(); }

// ====== GFX (avoid SPI ambiguity by not passing &SPI) ======
Arduino_DataBus *bus = new Arduino_HWSPI(PIN_DC, PIN_CS);
Arduino_GFX *gfx = new Arduino_ST7789(
  bus, PIN_RST, ROTATION, true,
  170, 320, COLSTART, ROWSTART, COLSTART, ROWSTART
);

// ====== Backlight / Haptics ======
static uint8_t gBrightness = 255;  // restored at setup
static inline void blSetHW(uint8_t brightness) { Breakout.analogWrite(PIN_BL, 255 - brightness); } // ACTIVE-LOW
static inline void vibOn(){  Breakout.digitalWrite(PIN_VIBR, VIB_ACTIVE_LOW ? LOW  : HIGH); }
static inline void vibOff(){ Breakout.digitalWrite(PIN_VIBR, VIB_ACTIVE_LOW ? HIGH : LOW ); }
static void vibrate(uint16_t ms){ vibOn(); delay(60); vibOff(); (void)ms; }

// ====== Buttons ======
struct BtnState { uint8_t pin; bool pressed; const char* name; };
BtnState buttons[5] = {
  { (uint8_t)BTN_UP,    false, "UP"    },
  { (uint8_t)BTN_DOWN,  false, "DOWN"  },
  { (uint8_t)BTN_LEFT,  false, "LEFT"  },
  { (uint8_t)BTN_RIGHT, false, "RIGHT" },
  { (uint8_t)BTN_OK,    false, "OK"    }
};
const uint16_t DEBOUNCE_MS = 25;
uint32_t lastChange[5] = {0,0,0,0,0};
static void setupButtons(){ pinMode(BTN_UP,INPUT); pinMode(BTN_DOWN,INPUT); pinMode(BTN_LEFT,INPUT); pinMode(BTN_RIGHT,INPUT); pinMode(BTN_OK,INPUT); }

// ====== Footer geometry ======
static bool footerInitDone=false;
static int  footerY=0;
static int  chipX=0, chipW=0, chipH=0;
static int  wifiX=0, wifiW=0;
static int  blX=0,   blW=0;

// ====== RTC / Time ======
WiFiUDP ntpUDP;
static const uint32_t RESYNC_INTERVAL_MS = 6UL*3600UL*1000UL;  // 6 hours
static uint32_t nextResyncAt = 0;

static inline void setTZ(){
  setenv("TZ","CST6CDT,M3.2.0/2,M11.1.0/2",1); tzset(); // America/Chicago
}
static bool rtcLooksValid(){
  time_t now = time(NULL);
  return (now >= 1704067200); // >= Jan 1, 2024 UTC
}

// ====== WiFi / NTP async state-machine ======
enum SyncState : uint8_t {
  SYNC_IDLE=0, SYNC_START, SYNC_WAIT_WIFI, SYNC_FAST_NTP, SYNC_DNS, SYNC_NTP_SEND, SYNC_NTP_WAIT, SYNC_HTTP_DATE
};
static SyncState syncState = SYNC_IDLE;
static uint32_t  syncStepStarted = 0;
static uint32_t  wifiDeadline = 0;
static IPAddress ntpIP;
static uint8_t   ntpRetries = 0;
static bool      wifiOn = false;

// pre-known time.google.com anycast IPs seen in your logs
static IPAddress kFastNtpIPs[] = {
  IPAddress(216,239,35,4),
  IPAddress(216,239,35,8)
};
static uint8_t kFastNtpIdx = 0;

static void wifiOff(){
  if (wifiOn){
    WiFi.disconnect();
    WiFi.end();
    wifiOn = false;
    Serial.println("[WiFi] disconnected");
  }
}
static void syncBegin(bool manual){
  if (rtcLooksValid() && !manual){
    nextResyncAt = millis() + RESYNC_INTERVAL_MS;
    Serial.println("[SYNC] skip (RTC already valid)");
    syncState = SYNC_IDLE;
    return;
  }
  Serial.println(manual ? "[SYNC] start manual" : "[SYNC] start boot/scheduled");
  WiFi.disconnect(); WiFi.end(); delay(5);
  Serial.println("[WiFi] begin...");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  wifiOn = true;
  wifiDeadline = millis() + 7000; // 7s cap
  syncState = SYNC_WAIT_WIFI; syncStepStarted = millis(); ntpRetries = 0; kFastNtpIdx = 0;
}
static bool tj_sendNtpToIp(const IPAddress &ip){
  const uint8_t NTP_PACKET_SIZE = 48;
  uint8_t pkt[NTP_PACKET_SIZE]; memset(pkt, 0, sizeof(pkt));
  pkt[0] = 0b11100011; pkt[2] = 6; pkt[3] = 0xEC; pkt[12]=49; pkt[13]=0x4E; pkt[14]=49; pkt[15]=52;
  if (!ntpUDP.beginPacket(ip, 123)) return false;
  ntpUDP.write(pkt, NTP_PACKET_SIZE);
  return ntpUDP.endPacket()==1;
}
static bool tj_parseNtpTime(time_t &outUTC){
  const uint8_t NTP_PACKET_SIZE = 48;
  if (ntpUDP.parsePacket() < (int)NTP_PACKET_SIZE) return false;
  uint8_t pkt[NTP_PACKET_SIZE]; ntpUDP.read(pkt, NTP_PACKET_SIZE);
  uint32_t secsSince1900 = ((uint32_t)pkt[40]<<24) | ((uint32_t)pkt[41]<<16) | ((uint32_t)pkt[42]<<8) | (uint32_t)pkt[43];
  const uint32_t seventyYears = 2208988800UL;
  if (secsSince1900 < seventyYears) return false;
  outUTC = (time_t)(secsSince1900 - seventyYears);
  return true;
}

// --- UTC converter (no _rtc_maketime, no timegm) ---
static time_t timegm_utc(const struct tm *t){
  int year  = t->tm_year + 1900;
  int month = t->tm_mon + 1;   // 1..12
  int day   = t->tm_mday;      // 1..31
  int a = (14 - month)/12;
  int y = year + 4800 - a;
  int m = month + 12*a - 3;
  int days = day + (153*m + 2)/5 + 365*y + y/4 - y/100 + y/400 - 32045;
  int days_since_epoch = days - 2440588;
  long long secs = (long long)days_since_epoch * 86400LL
                 + t->tm_hour * 3600LL + t->tm_min * 60LL + t->tm_sec;
  return (time_t)secs;
}

// --- HTTP Date fallback (TCP/80) ---
static bool parseHTTPDateToUnix(const char *dateLine, time_t &outUTC){
  char wk[4]={0}, mon[4]={0};
  int d=0,y=0,hh=0,mm=0,ss=0;
  int n = sscanf(dateLine, "Date: %3s, %d %3s %d %d:%d:%d GMT", wk, &d, mon, &y, &hh, &mm, &ss);
  if (n < 7) return false;
  const char *months="JanFebMarAprMayJunJulAugSepOctNovDec";
  const char *p=strstr(months, mon); if(!p) return false;
  int m = (int)((p - months)/3) + 1;
  struct tm t; memset(&t,0,sizeof(t));
  t.tm_year = y - 1900; t.tm_mon = m-1; t.tm_mday=d; t.tm_hour=hh; t.tm_min=mm; t.tm_sec=ss;
  outUTC = timegm_utc(&t);
  return (outUTC > 0);
}
static bool httpFetchDate(time_t &outUTC){
  const char* hosts[] = {"google.com","example.com","cloudflare.com"};
  WiFiClient client;
  for (uint8_t i=0;i<3;i++){
    const char* host = hosts[i];
    client.setTimeout(500);
    if (!client.connect(host, 80)){ continue; }
    client.print(String("HEAD / HTTP/1.1\r\nHost: ") + host + "\r\nConnection: close\r\n\r\n");
    uint32_t start = millis();
    while (client.connected() && (millis()-start) < 1200){
      String line = client.readStringUntil('\n');
      line.trim();
      if (line.startsWith("Date:")){
        Serial.print("[HTTP] "); Serial.println(line);
        time_t utc=0; if (parseHTTPDateToUnix(line.c_str(), utc)){ outUTC=utc; client.stop(); return true; }
      }
      if (line.length()==0) { break; }
    }
    client.stop();
  }
  return false;
}

static void syncPump(){
  switch(syncState){
    case SYNC_IDLE: break;
    case SYNC_WAIT_WIFI:{
      wl_status_t st = (wl_status_t)WiFi.status();
      if (st == WL_CONNECTED){
        Serial.print("[WiFi] IP="); Serial.println(WiFi.localIP());
        if (!ntpUDP.begin(2390)) { Serial.println("[NTP] UDP begin fail"); syncState=SYNC_HTTP_DATE; break; }
        syncState = SYNC_FAST_NTP; syncStepStarted=millis(); ntpRetries=0; kFastNtpIdx=0;
      } else if ((int32_t)(millis()-wifiDeadline) >= 0){
        Serial.println("[WiFi] connect timeout"); wifiOff(); syncState = SYNC_IDLE;
      }
    } break;
    case SYNC_FAST_NTP:{
      if (kFastNtpIdx < (sizeof(kFastNtpIPs)/sizeof(kFastNtpIPs[0]))){
        IPAddress ip = kFastNtpIPs[kFastNtpIdx++];
        if (tj_sendNtpToIp(ip)){ syncState = SYNC_NTP_WAIT; syncStepStarted = millis(); ntpIP = ip; }
        else { /* try next on next tick */ }
      } else {
        syncState = SYNC_DNS; syncStepStarted=millis();
      }
    } break;
    case SYNC_DNS:{
      if (WiFi.hostByName("time.google.com", ntpIP) != 1){
        Serial.println("[DNS] failed -> HTTP fallback"); ntpUDP.stop(); syncState = SYNC_HTTP_DATE;
      } else {
        Serial.print("[DNS] time.google.com -> "); Serial.println(ntpIP);
        syncState = SYNC_NTP_SEND; syncStepStarted=millis(); ntpRetries=0;
      }
    } break;
    case SYNC_NTP_SEND:{
      if (!tj_sendNtpToIp(ntpIP)){ Serial.println("[NTP] send fail -> HTTP fallback"); ntpUDP.stop(); syncState = SYNC_HTTP_DATE; break; }
      syncState = SYNC_NTP_WAIT; syncStepStarted = millis();
    } break;
    case SYNC_NTP_WAIT:{
      time_t utc=0;
      if (tj_parseNtpTime(utc)){
        struct timeval tv; tv.tv_sec = utc; tv.tv_usec=0; settimeofday(&tv, nullptr);
        saveTimeFS(utc);
        setenv("TZ","CST6CDT,M3.2.0/2,M11.1.0/2",1); tzset();
        time_t now=time(NULL); struct tm *lt=localtime(&now);
        Serial.print("[NTP] OK UTC="); Serial.print((unsigned long)utc);
        Serial.print(" Local "); Serial.print(lt->tm_mon+1); Serial.print('/'); Serial.print(lt->tm_mday);
        Serial.print(' '); Serial.print(lt->tm_hour); Serial.print(':'); Serial.println(lt->tm_min);
        nextResyncAt = millis() + 6UL*3600UL*1000UL;
        ntpUDP.stop(); wifiOff(); syncState = SYNC_IDLE;
      } else if ((millis() - syncStepStarted) > 1200){
        if (++ntpRetries < 2){ syncState = (kFastNtpIdx<(sizeof(kFastNtpIPs)/sizeof(kFastNtpIPs[0])))? SYNC_FAST_NTP : SYNC_NTP_SEND; }
        else { Serial.println("[NTP] timeout -> HTTP fallback"); ntpUDP.stop(); syncState = SYNC_HTTP_DATE; }
      }
    } break;
    case SYNC_HTTP_DATE:{
      time_t utc=0;
      if (httpFetchDate(utc)){
        struct timeval tv; tv.tv_sec = utc; tv.tv_usec=0; settimeofday(&tv, nullptr);
        saveTimeFS(utc);
        setenv("TZ","CST6CDT,M3.2.0/2,M11.1.0/2",1); tzset();
        time_t now=time(NULL); struct tm *lt=localtime(&now);
        Serial.print("[HTTP] OK UTC="); Serial.print((unsigned long)utc);
        Serial.print(" Local "); Serial.print(lt->tm_mon+1); Serial.print('/'); Serial.print(lt->tm_mday);
        Serial.print(' '); Serial.print(lt->tm_hour); Serial.print(':'); Serial.println(lt->tm_min);
        nextResyncAt = millis() + 6UL*3600UL*1000UL;
      } else {
        Serial.println("[HTTP] Date fetch failed");
      }
      ntpUDP.stop(); wifiOff(); syncState = SYNC_IDLE;
    } break;
  }
}

// ====== Layout helpers ======
struct BoxPx { int x; int y; };
BoxPx   gBoxTL, gBoxTR, gBoxBL, gBoxBR;

static void computeLayoutPx() {
  const int BOX_W_PX   = 12 * CHAR_W;     // 144
  const int TOP_H_PX   = 3  * CHAR_H;
  const int BOT_H_PX   = 4  * CHAR_H;
  const int W=320, H=170;
  int x0 = 8;
  int x1 = W - 8 - BOX_W_PX;
  const int TOP = 24;
  const int BOT = 24;
  const int MIDG = H - TOP - BOT - TOP_H_PX - BOT_H_PX;
  int yTop = TOP;
  int yBot = TOP + TOP_H_PX + MIDG;
  gBoxTL = { x0, yTop };
  gBoxTR = { x1, yTop };
  gBoxBL = { x0, yBot };
  gBoxBR = { x1, yBot };
  // Footer geometry
  footerY = H - CHAR_H - 2;
  const char *LOGO="N1LABS";
  int textW=(int)strlen(LOGO)*CHAR_W; int padX=5, padY=1;
  chipW = textW + padX*2; chipH = CHAR_H + padY*2 - 1; chipX = 8;
  wifiW = (int)strlen("[W OFF]") * CHAR_W;
  blW   = (int)strlen("[BL 100%]") * CHAR_W;
  wifiX = (W - wifiW)/2;
  blX   = W - 6 - blW;
}

static void drawTitleOnce(){
  gfx->setTextColor(COL_TEXT, COL_BG); gfx->setTextSize(2);
  gfx->setCursor(8,2); gfx->print("Portenta H7");
}

// Boxes
static void drawNiceBox(int x, int y, int wChars, const char* label, const char* value) {
  if (wChars < 8) return;
  const int inner = wChars - 2;
  static char line1[64], line2[64], line3[64];
  int labLen = (int)strlen(label); if (labLen > inner - 2) labLen = inner - 2;
  int coreLen = labLen + 2; int rem = inner - coreLen;
  int leftFill = rem/2, rightFill = rem - leftFill;
  int p=0; line1[p++]='+'; for(int i=0;i<leftFill;i++) line1[p++]='-';
  line1[p++]='['; for(int i=0;i<labLen;i++) line1[p++]=label[i]; line1[p++]=']';
  for(int i=0;i<rightFill;i++) line1[p++]='-'; line1[p++]='+'; line1[p]=0;
  int valLen = (int)strlen(value); if (valLen > inner) valLen = inner;
  int space = inner - valLen; int lpad = space/2; int rpad = space - lpad;
  p=0; line2[p++]='|'; for(int i=0;i<lpad;i++) line2[p++]=' ';
  for(int i=0;i<valLen;i++) line2[p++]=value[i];
  for(int i=0;i<rpad;i++) line2[p++]=' '; line2[p++]='|'; line2[p]=0;
  p=0; line3[p++]='+'; for(int i=0;i<inner;i++) line3[p++]='-'; line3[p++]='+'; line3[p]=0;
  gfx->setTextColor(COL_TEXT, COL_BG); gfx->setTextSize(2);
  gfx->setCursor(x, y);                         gfx->print(line1);
  gfx->setCursor(x, y + CHAR_H + VPAD);         gfx->print(line2);
  gfx->setCursor(x, y + 2*CHAR_H + 2*VPAD);     gfx->print(line3);
}

static void drawInfoBox(int x, int y, int wChars, const char* label, const char* lineA, const char* lineB) {
  if (wChars < 8) return;
  const int inner = wChars - 2;
  static char top[64], mid1[64], mid2[64], bot[64];
  int labLen = (int)strlen(label); if (labLen > inner - 2) labLen = inner - 2;
  int coreLen = labLen + 2, rem = inner - coreLen;
  int leftFill = rem/2, rightFill = rem - leftFill;
  int p=0; top[p++]='+'; for(int i=0;i<leftFill;i++) top[p++]='-';
  top[p++]='['; for(int i=0;i<labLen;i++) top[p++]=label[i]; top[p++]=']';
  for(int i=0;i<rightFill;i++) top[p++]='-'; top[p++]='+'; top[p]=0;
  auto mid = [&](char* dst, const char* txt){
    int tlen = (int)strlen(txt); if (tlen > inner) tlen = inner;
    int lpad = 1, rpad = inner - lpad - tlen; if (rpad < 0) rpad = 0;
    int q=0; dst[q++]='|'; dst[q++]=' ';
    for(int i=0;i<tlen;i++) dst[q++]=txt[i];
    for(int i=0;i<rpad;i++) dst[q++]=' ';
    dst[q++]='|'; dst[q]=0;
  };
  mid(mid1, lineA);
  mid(mid2, lineB);
  p=0; bot[p++]='+'; for(int i=0;i<inner;i++) bot[p++]='-'; bot[p++]='+'; bot[p]=0;
  gfx->setTextColor(COL_TEXT, COL_BG); gfx->setTextSize(2);
  gfx->setCursor(x, y);                           gfx->print(top);
  gfx->setCursor(x, y + CHAR_H + VPAD);           gfx->print(mid1);
  gfx->setCursor(x, y + 2*CHAR_H + VPAD);         gfx->print(mid2);
  gfx->setCursor(x, y + 3*CHAR_H + 2*VPAD);       gfx->print(bot);
}

// ====== Footer ======
static int lastBLpct=-1;
static bool wifiShownOn=false;

static void drawFooterWiFi(bool on){
  const char *t = on ? "[W ON]" : "[W OFF]";
  gfx->fillRect(wifiX, footerY-1, wifiW, CHAR_H+2, COL_BG);
  gfx->setTextColor(COL_TEXT, COL_BG); gfx->setTextSize(2); gfx->setCursor(wifiX, footerY); gfx->print(t);
  wifiShownOn = on;
}
static void drawFooterBLIfChanged(){
  int pct = (int)gBrightness * 100 / 255;
  if (!footerInitDone || pct==lastBLpct) return;
  lastBLpct=pct;
  char txt[20]; snprintf(txt,sizeof(txt),"[BL %d%%]", pct);
  gfx->fillRect(blX, footerY-1, blW, CHAR_H+2, COL_BG);
  gfx->setTextColor(COL_TEXT, COL_BG); gfx->setTextSize(2); gfx->setCursor(blX, footerY); gfx->print(txt);
}
static void drawFooterInit(){
  // whole footer strip
  gfx->fillRect(0, footerY-1, 320, CHAR_H+3, COL_BG);
  // Left chip
  const char *LOGO="N1LABS"; int textW=(int)strlen(LOGO)*CHAR_W; int padX=5,padY=1;
  int chipY = footerY-1, radius=2;
  gfx->fillRoundRect(chipX, chipY, chipW, chipH, radius, COL_CHIP);
  gfx->setTextColor(COL_CHIPFG, COL_CHIP); gfx->setTextSize(2); gfx->setCursor(chipX+padX, footerY); gfx->print(LOGO);
  // Center WiFi placeholder + Right BL
  drawFooterWiFi(false);
  lastBLpct=-1; drawFooterBLIfChanged();
  footerInitDone=true;
}
static void drawFooterMaintain(){
  if (!footerInitDone) drawFooterInit();
  drawFooterBLIfChanged();
  bool on = (WiFi.status()==WL_CONNECTED) || (syncState==SYNC_WAIT_WIFI || syncState==SYNC_FAST_NTP || syncState==SYNC_DNS || syncState==SYNC_NTP_SEND || syncState==SYNC_NTP_WAIT || syncState==SYNC_HTTP_DATE);
  if (on != wifiShownOn) drawFooterWiFi(on);
}

// ====== Clock chips (Time + Date) ======
static char gPrevClk[32]={0};
const int CLK_Y = 2;
const int CLK_AREA_W = 14 * CHAR_W;
const int CLK_AREA_X = 320 - 8 - CLK_AREA_W;
static void drawClockChips(bool force){
  char tbuf[16], dbuf[16], combined[32];
  time_t now=time(NULL); struct tm *t=localtime(&now);
  int hr12=t->tm_hour%12; if(hr12==0)hr12=12; const char* ap=(t->tm_hour < 12)?"AM":"PM";
  snprintf(tbuf,sizeof(tbuf),"%d:%02d %s", hr12, t->tm_min, ap);
  snprintf(dbuf,sizeof(dbuf),"%d/%d", t->tm_mon+1, t->tm_mday);
  snprintf(combined,sizeof(combined),"%s|%s", tbuf, dbuf);
  if(!force && strcmp(combined,gPrevClk)==0) return;
  strncpy(gPrevClk,combined,sizeof(gPrevClk)); gPrevClk[sizeof(gPrevClk)-1]=0;
  gfx->fillRect(CLK_AREA_X, 0, CLK_AREA_W, CHAR_H+6, COL_BG);
  int padX=6, padY=1, radius=2;
  int tw=strlen(tbuf)*CHAR_W, dw=strlen(dbuf)*CHAR_W;
  int w1=tw+padX*2, w2=dw+padX*2, chipH=CHAR_H+padY*2-1, overlap=radius;
  int total=w1+w2-overlap; int x=CLK_AREA_X+(CLK_AREA_W-total); int y=CLK_Y-1;
  gfx->fillRoundRect(x, y, w1, chipH, radius, COL_TIME_BG);
  gfx->setTextColor(COL_CLK_TXT, COL_TIME_BG); gfx->setTextSize(2); gfx->setCursor(x+padX, CLK_Y); gfx->print(tbuf);
  int x2=x+w1-overlap;
  gfx->fillRoundRect(x2, y, w2, chipH, radius, COL_DATE_BG);
  gfx->setTextColor(COL_CLK_TXT, COL_DATE_BG); gfx->setCursor(x2+padX, CLK_Y); gfx->print(dbuf);
}

// ====== Fade helpers (brief, acceptable blocking during transition) ======
static void fadeTo(uint8_t target, uint16_t ms){
  uint8_t start = gBrightness;
  uint32_t t0 = millis();
  while (true){
    float f = (millis()-t0) / (float)ms; if (f<0) f=0; if (f>1) f=1;
    int cur = (int)(start + (target - start)*f);
    blSetHW((uint8_t)cur);
    if (f>=1) break;
    delay(8);
  }
}
static void fadeOutInForHome(){
  uint8_t orig = gBrightness;
  fadeTo(0, 250);
  // caller will draw Home screen here (with backlight off)
  // fade back up
  fadeTo(orig, 250);
}

// ====== Splash (5s, then fade out) ======
static void showSplash5s(){
  gfx->fillScreen(COL_BG);
  gfx->setTextColor(COL_TEXT, COL_BG);
  const char *label = "N1LABS";
  const int cw3 = 18, ch3 = 24;
  int lw = (int)strlen(label) * cw3;
  int lx = (320 - lw)/2;
  int ly = (170 - ch3)/2 - 12;
  gfx->setTextSize(3); gfx->setCursor(lx, ly); gfx->print(label);
  const int cw2 = 12, ch2 = 16;
  const int barChars = 14;
  int bar_px = (barChars + 2) * cw2;
  int x = (320 - bar_px) / 2;
  int y = ly + ch3 + 4;
  gfx->setTextSize(2);
  gfx->setCursor(x, y); gfx->print('[');
  for (int i=0;i<barChars;i++) gfx->print(' ');
  gfx->print(']');
  const char *msgTemplate = "Initializing... 100%";
  int msg_w = (int)strlen(msgTemplate) * cw2;
  int msg_x = (320 - msg_w)/2;
  int msg_y = y + ch2 + 2;
  int last_filled = 0;
  int last_pct = -1;
  uint32_t start = millis();
  while (millis() - start < 5000){
    float frac = (float)(millis() - start) / 5000.0f; if (frac<0) frac=0; if (frac>1) frac=1;
    int filled = (int)floorf(frac * barChars + 0.5f);
    int pct    = (int)roundf(frac * 100.0f);
    if (filled > last_filled){
      for (int i = last_filled; i < filled; ++i){
        gfx->setCursor(x + cw2*(1 + i), y); gfx->print('=');
      }
      last_filled = filled;
    }
    if (pct != last_pct){
      char msg[32]; snprintf(msg, sizeof(msg), "Initializing... %3d%%", pct);
      gfx->setCursor(msg_x, msg_y); gfx->print(msg); last_pct = pct;
    }
    delay(30);
  }
  // Fade out to transition to Home
  fadeTo(0, 250);
}

// ====== M7 CPU load ======
#if __has_include("mbed_stats.h")
  #include "mbed_stats.h"
  #define HAS_MBED_CPU_STATS 1
#else
  #define HAS_MBED_CPU_STATS 0
#endif
static inline int m7MHz(){ return (int)(SystemCoreClock/1000000UL); }
static float m7CpuLoadPct(bool &ready){
  static uint64_t lastIdle=0,lastUp=0; ready=false;
#if HAS_MBED_CPU_STATS
  mbed_stats_cpu_t s; mbed_stats_cpu_get(&s);
  if(lastUp==0){ lastIdle=s.idle_time; lastUp=s.uptime; return 0.0f; }
  uint64_t dIdle=(s.idle_time>=lastIdle)?(s.idle_time-lastIdle):0;
  uint64_t dUp  =(s.uptime   >=lastUp )?(s.uptime   -lastUp ):0;
  lastIdle=s.idle_time; lastUp=s.uptime; if(dUp==0) return 0.0f;
  float load=100.0f*(1.0f-(float)dIdle/(float)dUp);
  if(load<0)load=0; if(load>100)load=100; ready=true; return load;
#else
  return 0.0f;
#endif
}

// ====== M4 link + stats via RPC ======
volatile int gM4LoadPct = -1;
volatile int gM4MHz     = 200;
static uint32_t gM4LastMs = 0;
static bool m4AutoBootTried = false;
static bool postTasksStarted=false;
static uint32_t postTasksAt=0;

// non-blocking line reader for RPC
static String gRpcBuf;
static void rpcPumpNonBlocking(){
  while (RPC.available()){
    char c = (char)RPC.read();
    if (c=='\r') continue;
    if (c=='\n'){
      String line = gRpcBuf; gRpcBuf="";
      line.trim();
      if (line.length()){
        // handle
        if (line.startsWith("M4,")) {
          char buf[64]; line.toCharArray(buf, sizeof(buf));
          int load= -1, mhz = 0;
          if (2 == sscanf(buf, "M4,%d,%d", &load, &mhz)) {
            if (load < 0) load = -1; if (load > 100) load = 100;
            if (mhz >= 1800 && mhz <= 2200) mhz /= 10;  // fix "2004"
            if (mhz < 0 || mhz > 600) mhz = 200;
            gM4LoadPct = load; gM4MHz = mhz; gM4LastMs = millis();
          }
        } else if (line.startsWith("M4:HB") || line.startsWith("M4:READY") || line.startsWith("M4:PONG")) {
          gM4LastMs = millis();
        }
      }
    } else {
      if (gRpcBuf.length() < 120) gRpcBuf += c;
    }
  }
}

static void drawMidLinkX(){
  const int cx = (gBoxBL.x + (gBoxBR.x+12*CHAR_W))/2 - CHAR_W/2;
  const int cy = gBoxBL.y + (2*CHAR_H + VPAD);
  bool linked = (millis() - gM4LastMs) < 2000;
  uint16_t c = linked ? COL_CHIP : COL_BADLINK;
  gfx->setTextColor(c, COL_BG); gfx->setTextSize(2);
  gfx->setCursor(cx, cy); gfx->print('x');
}

// ====== Brightness helpers ======
static void adjustBrightness(int d){
  int v=(int)gBrightness+d; v=constrain(v,0,255);
  if (v!=gBrightness){ gBrightness=(uint8_t)v; blSetHW(gBrightness); saveBrightnessFS(gBrightness); drawFooterBLIfChanged(); }
}

// ====== OK long-press for manual sync (non-blocking) ======
static bool okWasDown=false; static uint32_t okDownMs=0;
static void pollOkLongPress(){
  bool down = (digitalRead(BTN_OK)==LOW);
  uint32_t now=millis();
  if (down && !okWasDown){ okWasDown=true; okDownMs=now; }
  else if (!down && okWasDown){ okWasDown=false; }
  if (down && okWasDown && (now-okDownMs>700)){
    Serial.println("[SYNC] manual trigger (OK long-press)");
    vibrate(60);
    syncBegin(true);
    okWasDown=false;
  }
}

// ====== Helpers ======
static void fmtUptime(char *out, size_t n){
  uint32_t ms = millis();
  uint32_t sec = ms / 1000;
  uint32_t days = sec / 86400;  sec %= 86400;
  uint32_t hrs  = sec / 3600;   sec %= 3600;
  uint32_t mins = sec / 60;     sec %= 60;
  if (days > 0) snprintf(out, n, "%lud%02luh", (unsigned long)days, (unsigned long)hrs);
  else          snprintf(out, n, "%02lu:%02lu:%02lu", (unsigned long)hrs, (unsigned long)mins, (unsigned long)sec);
}

// ====== SETUP / LOOP ======
void setup() {
  Serial.begin(115200);

  storageBegin();
  uint8_t savedBL=0; if (restoreBrightnessFS(savedBL)) gBrightness = savedBL;
  Breakout.pinMode(PIN_BL, OUTPUT); blSetHW(gBrightness);

  time_t tSaved=0; if (!rtcLooksValid() && restoreTimeFS(tSaved)){ struct timeval tv; tv.tv_sec = tSaved; tv.tv_usec=0; settimeofday(&tv, nullptr); }
  setTZ();
  if (rtcLooksValid()) nextResyncAt = millis() + RESYNC_INTERVAL_MS;

  Breakout.pinMode(PIN_VIBR, OUTPUT); vibOff();
  setupButtons();
  BoardTemp_begin();

  gfx->begin(48000000);
  gfx->invertDisplay(INVERT_COLORS);
  gfx->setTextWrap(false); gfx->setTextSize(2); gfx->setTextColor(COL_TEXT, COL_BG);
  computeLayoutPx();

  showSplash5s(); // ends with fade to black

  // Draw Home with backlight off, then fade back in
  gfx->fillScreen(COL_BG);
  computeLayoutPx();
  drawTitleOnce(); drawClockChips(true); 
  drawNiceBox(gBoxTL.x, gBoxTL.y, 12, "Temp", "--");
  drawNiceBox(gBoxTR.x, gBoxTR.y, 12, "UPT",  "00:00:00");
  drawInfoBox(gBoxBL.x, gBoxBL.y, 12, "M7", "L: --", "F: 480MHz");
  drawInfoBox(gBoxBR.x, gBoxBR.y, 12, "M4", "L: OFF", "F: 200MHz");
  drawMidLinkX();
  // footer before fade-in so text is ready
  footerInitDone=false; drawFooterInit();
  // fade back to saved brightness
  fadeTo(gBrightness, 250);

  postTasksAt = millis() + 50;
}

void loop() {
  if (!postTasksStarted && (int32_t)(millis()-postTasksAt) >= 0){
    postTasksStarted = true;
    RPC.begin();
    bootM4();
    Serial.println("[RPC] boot M4");
    m4AutoBootTried = false;
    if (!rtcLooksValid()) syncBegin(false);
  }

  BoardTemp_poll_1Hz();

  // Buttons
  for (uint8_t i=0;i<5;i++){
    BtnState &b = buttons[i];
    bool raw=(digitalRead(b.pin)==LOW); uint32_t now=millis();
    if(raw!=b.pressed && (now-lastChange[i])>DEBOUNCE_MS){
      b.pressed=raw; lastChange[i]=now;
      if(b.pressed){
        vibOn(); delay(50); vibOff();
        if(b.pin==BTN_UP)   adjustBrightness(+16);
        if(b.pin==BTN_DOWN) adjustBrightness(-16);
        if(b.pin==BTN_LEFT) {
          Serial.println("[RPC] manual M4 reboot");
          bootM4();
          delay(30);
          RPC.end();
          delay(20);
          RPC.begin();
        }
      }
    }
  }
  pollOkLongPress();

  // Periodic ping to help liveness
  static uint32_t lastPing=0;
  if ((millis()-lastPing) > 1000){
    RPC.println("PING");
    lastPing = millis();
  }

  // RPC pump (non-blocking)
  rpcPumpNonBlocking();

  // If M4 stayed silent for ~3s after first boot, try once more
  static uint32_t firstBootAt = 0;
  if (!firstBootAt && postTasksStarted) firstBootAt = millis();
  if (postTasksStarted && !m4AutoBootTried && (millis()-firstBootAt)>3000 && (millis()-gM4LastMs)>2500){
    Serial.println("[RPC] auto-boot M4 (no HB)");
    bootM4();
    m4AutoBootTried = true;
  }

  if (nextResyncAt && (int32_t)(millis()-nextResyncAt) >= 0 && syncState==SYNC_IDLE){
    syncBegin(false);
  }

  syncPump();

  static int lastSec=-1; time_t now=time(NULL); struct tm *t=localtime(&now);
  if (t->tm_sec!=lastSec){ lastSec=t->tm_sec; drawClockChips(false); }

  static uint32_t tmo=0;
  if (millis()-tmo > 300) {
    tmo = millis();

    char temp[16] = "--";
    int __c = BoardTemp_getC();
    if (__c != -999) snprintf(temp, sizeof(temp), "%dC", __c);
    drawNiceBox(gBoxTL.x, gBoxTL.y, 12, "Temp", temp);

    char upt[16]; fmtUptime(upt, sizeof(upt));
    drawNiceBox(gBoxTR.x, gBoxTR.y, 12, "UPT", upt);

    bool ready=false; float load = m7CpuLoadPct(ready);
    int pct = ready ? (int)roundf(load) : -1;
    char a[24], b[24];
    if (pct < 0) snprintf(a,sizeof(a),"L: OFF"); else snprintf(a,sizeof(a),"L: %d%%", pct);
    snprintf(b,sizeof(b),"F: %dMHz", m7MHz());
    drawInfoBox(gBoxBL.x, gBoxBL.y, 12, "M7", a, b);

    if ((millis() - gM4LastMs) > 2000) { gM4LoadPct = -1; }
    if (gM4LoadPct < 0) snprintf(a,sizeof(a),"L: OFF"); else snprintf(a,sizeof(a),"L: %d%%", gM4LoadPct);
    snprintf(b,sizeof(b),"F: %dMHz", gM4MHz);
    drawInfoBox(gBoxBR.x, gBoxBR.y, 12, "M4", a, b);

    drawMidLinkX();
    drawFooterMaintain();
  }

  delay(1);
}
