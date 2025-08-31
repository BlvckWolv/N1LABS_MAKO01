/*
  Portenta H7 + Breakout + ST7789 (170x320)
  ASCII 2x2 Dashboard + RTC + M7/M4 CPU via RPC
  BG #444a7d, UP/DOWN = backlight, OK = buzz, LEFT = reboot M4, RIGHT = stress-load M4.
  Footer: "N1LABS" chip (green, radius 2) left, "BL:%" right.
  Time+Date (top-right): TWO touching chips (radius 2), bottom pulled down 1px for the date chip.
  Temp now shown in °C.
*/

#include <Arduino.h>
#include <Arduino_PortentaBreakout.h>
#include <SPI.h>
#include <Arduino_GFX_Library.h>
#include <string.h>
#include <math.h>
#include "mbed.h"
#include <time.h>
#include <RPC.h>
#include <Wire.h>

// ================= BoardTemp (MAX17262) — inline (no extra files) =================
// Reads MAX17262 (I2C 0x36) TEMP (0x08) and fallback DIETEMP (0x34).
// Poll at ~1 Hz. Returns Celsius as int via BoardTemp_getC(); BoardTemp_getF() kept for compat.
namespace BoardTempInline {
  static const uint8_t ADDR = 0x36;
  static const uint8_t REG_TEMP    = 0x08; // signed, 1/256 °C
  static const uint8_t REG_DIETEMP = 0x34; // signed, 1/256 °C
  static TwoWire &W = Wire1;
  static int       lastC  = -999;
  static int       lastF  = -999;
  static uint32_t  lastMs = 0;
  static bool      ready  = false;

  static bool i2c_read16_le(uint8_t reg, uint16_t &out){
    W.beginTransmission(ADDR);
    W.write(reg);
    if (W.endTransmission(false) != 0) return false; // repeated START
    int n = W.requestFrom((int)ADDR, 2);
    if (n != 2) return false;
    uint8_t lo = W.read(), hi = W.read();
    out = (uint16_t)((hi<<8)|lo);
    return true;
  }
  static bool readC_from(uint8_t reg, float &outC){
    uint16_t raw;
    if (!i2c_read16_le(reg, raw)) return false;
    int16_t s = (int16_t)raw;
    outC = (float)s / 256.0f;
    return true;
  }
  void begin(){ W.begin(); W.setClock(400000); lastC = lastF = -999; lastMs = 0; ready = true; }
  void poll_1Hz(){
    if (!ready) return;
    uint32_t now = millis();
    if (now - lastMs < 1000) return;
    lastMs = now;
    float c = NAN;
    bool ok = readC_from(REG_TEMP, c);        // battery/gauge temp first
    if (!ok) ok = readC_from(REG_DIETEMP, c); // fallback to die temp
    if (ok && isfinite(c) && c > -40.0f && c < 125.0f){
      lastC = (int)lroundf(c);
      lastF = (int)lroundf(c * 9.0f / 5.0f + 32.0f);
    } else {
      lastC = lastF = -999;
    }
  }
  int getC(){ return lastC; }
  int getF(){ return lastF; }
}
// Aliases expected by the rest of the sketch:
static inline void BoardTemp_begin(){ BoardTempInline::begin(); }
static inline void BoardTemp_poll_1Hz(){ BoardTempInline::poll_1Hz(); }
static inline int  BoardTemp_getC(){ return BoardTempInline::getC(); }
static inline int  BoardTemp_getF(){ return BoardTempInline::getF(); }
// ================================================================================

// ===== Display/pins =====
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

const int BTN_RIGHT = GPIO_3;  // stress-load M4
const int BTN_DOWN  = GPIO_4;
const int BTN_LEFT  = GPIO_5;  // manual M4 reboot
const int BTN_UP    = GPIO_6;
const int BTN_OK    = GPIO_0;

#define C565(r,g,b) ( ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b) >> 3) )
const uint16_t COL_BG     = C565(68, 74, 125);   // #444a7d
const uint16_t COL_TEXT   = C565(235,239,245);
const uint16_t COL_CHIP   = C565(166,218,149);   // #a6da95 (N1LABS)
const uint16_t COL_CHIPFG = COL_BG;              // dark text
const uint16_t COL_TIME_BG = C565(183,189,248);  // #b7bdf8
const uint16_t COL_DATE_BG = C565(138,173,244);  // #8aadf4
const uint16_t COL_CLK_TXT = COL_BG;             // dark text

const int CHAR_W = 12; // font size 2
const int CHAR_H = 16;
const int VPAD   = 2;

struct BtnState { uint8_t pin; bool pressed; const char* name; };
struct BoxPx   { int x; int y; };

// ---- layout boxes ----
BoxPx   gBoxTL, gBoxTR, gBoxBL, gBoxBR;

BtnState buttons[5] = {
  { (uint8_t)BTN_UP,    false, "UP"    },
  { (uint8_t)BTN_DOWN,  false, "DOWN"  },
  { (uint8_t)BTN_LEFT,  false, "LEFT"  },
  { (uint8_t)BTN_RIGHT, false, "RIGHT" },
  { (uint8_t)BTN_OK,    false, "OK"    }
};
const uint16_t DEBOUNCE_MS = 25;
uint32_t lastChange[5] = {0,0,0,0,0};

uint8_t gBrightness = 220;

// From M4
volatile int gM4LoadPct = -1;
volatile int gM4MHz     = 0;
uint32_t     gM4LastMs  = 0;

// M4 control flags
volatile bool gRebootM4Requested = false;
volatile bool gSpinM4Requested   = false;
static uint32_t gSpinStopAtMs = 0;  // for TESTLOAD fallback stop

// GFX
Arduino_DataBus *bus = new Arduino_HWSPI(PIN_DC, PIN_CS, &SPI);
Arduino_GFX *gfx = new Arduino_ST7789(
  bus, PIN_RST, ROTATION, true,
  170, 320, COLSTART, ROWSTART, COLSTART, ROWSTART
);

// ==== M7 CPU load ====
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

// ==== Backlight / Haptics ====
static inline void blSet(uint8_t brightness) { Breakout.analogWrite(PIN_BL, 255 - brightness); } // ACTIVE-LOW
static inline void vibOn(){  Breakout.digitalWrite(PIN_VIBR, VIB_ACTIVE_LOW ? LOW  : HIGH); }
static inline void vibOff(){ Breakout.digitalWrite(PIN_VIBR, VIB_ACTIVE_LOW ? HIGH : LOW ); }
static void vibrate(uint16_t ms){ vibOn(); delay(ms); vibOff(); }

// ===== ASCII boxes =====
static void drawNiceBox(int x, int y, int wChars, const char* label, const char* value) {
  if (wChars < 8) return;
  const int inner = wChars - 2;
  static char line1[64], line2[64], line3[64];

  int labLen = (int)strlen(label); if (labLen > inner - 2) labLen = inner - 2;
  int coreLen = labLen + 2, rem = inner - coreLen;
  int leftFill = rem/2, rightFill = rem - leftFill;

  int p=0; line1[p++]='+'; for(int i=0;i<leftFill;i++) line1[p++]='-';
  line1[p++]='['; for(int i=0;i<labLen;i++) line1[p++]=label[i]; line1[p++]=']';
  for(int i=0;i<rightFill;i++) line1[p++]='-'; line1[p++]='+'; line1[p]=0;

  int valLen = (int)strlen(value); if (valLen > inner) valLen = inner;
  int space = inner - valLen, lpad = space/2, rpad = space - lpad;
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

// ===== Layout (2x2) =====
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
}

// ===== Title =====
static void drawTitleOnce(){
  gfx->setTextColor(COL_TEXT, COL_BG); gfx->setTextSize(2);
  gfx->setCursor(8,2); gfx->print("Portenta H7");
}

// ===== Clock chips (Time + Date as adjacent pills) =====
static char gPrevClk[32]={0};
const int CLK_Y = 2;
const int CLK_AREA_W = 14 * CHAR_W;   // budget area at top-right
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

  int padX = 6, padY = 1;
  int radius = 2;
  int tw = strlen(tbuf)*CHAR_W;
  int dw = strlen(dbuf)*CHAR_W;
  int w1 = tw + padX*2;
  int w2 = dw + padX*2;
  int chipH  = CHAR_H + padY*2 - 1; // +1px lower
  int overlap = radius;
  int total = w1 + w2 - overlap;
  int x = CLK_AREA_X + (CLK_AREA_W - total);
  int y = CLK_Y - 1;

  gfx->fillRoundRect(x, y, w1, chipH, radius, COL_TIME_BG);
  gfx->setTextColor(COL_CLK_TXT, COL_TIME_BG);
  gfx->setTextSize(2);
  gfx->setCursor(x + padX, CLK_Y);
  gfx->print(tbuf);

  int x2 = x + w1 - overlap;
  gfx->fillRoundRect(x2, y, w2, chipH, radius, COL_DATE_BG);
  gfx->setTextColor(COL_CLK_TXT, COL_DATE_BG);
  gfx->setCursor(x2 + padX, CLK_Y);
  gfx->print(dbuf);
}

// Footer with left "N1LABS" chip and right BL:%
static bool footerInitDone=false;
static int  lastBLpct=-1;

static void drawFooterInit(){
  const int y = 170 - CHAR_H - 2;
  gfx->fillRect(0, y, 320, CHAR_H+2, COL_BG);

  const char *LOGO = "N1LABS";
  int textW = (int)strlen(LOGO) * CHAR_W;
  int padX  = 5, padY = 1;
  int chipW = textW + padX*2;
  int chipH = CHAR_H + padY*2 - 2;
  int chipX = 8;
  int chipY = y - 1;
  int radius = 2;

  gfx->fillRoundRect(chipX, chipY, chipW, chipH, radius, COL_CHIP);
  gfx->setTextColor(COL_CHIPFG, COL_CHIP);
  gfx->setTextSize(2);
  gfx->setCursor(chipX + padX, y);
  gfx->print(LOGO);

  footerInitDone=true;
  lastBLpct=-1;
}

static void drawFooterBLIfChanged(){
  const int y = 170 - CHAR_H - 2;
  int pct=(int)gBrightness*100/255;
  if(pct==lastBLpct && footerInitDone) return;
  lastBLpct=pct;

  char buf[16]; snprintf(buf,sizeof(buf),"BL:%d%%",pct);
  int16_t tw=strlen(buf)*CHAR_W;
  gfx->fillRect(320-8-tw-2, y, tw+2, CHAR_H+2, COL_BG);
  gfx->setTextColor(COL_TEXT,COL_BG); gfx->setTextSize(2);
  gfx->setCursor(320-8-tw,y); gfx->print(buf);
}

static void drawFooterOnceThenMaintain(){ if(!footerInitDone) drawFooterInit(); drawFooterBLIfChanged(); }

// ===== Buttons =====
static void setupButtons(){ pinMode(BTN_UP,INPUT); pinMode(BTN_DOWN,INPUT); pinMode(BTN_LEFT,INPUT); pinMode(BTN_RIGHT,INPUT); pinMode(BTN_OK,INPUT); }
static void adjustBrightness(int d){ int v=(int)gBrightness+d; v=constrain(v,0,255); if(v!=gBrightness){ gBrightness=(uint8_t)v; blSet(gBrightness); drawFooterBLIfChanged(); } }
static void updateButton(BtnState &b, uint8_t i){
  bool raw=(digitalRead(b.pin)==LOW); uint32_t now=millis();
  if(raw!=b.pressed && (now-lastChange[i])>DEBOUNCE_MS){
    b.pressed=raw; lastChange[i]=now;
    if(b.pressed){
      vibOn(); delay(120); vibOff();
      if(b.pin==BTN_UP)   adjustBrightness(+16);
      if(b.pin==BTN_DOWN) adjustBrightness(-16);
      if(b.pin==BTN_LEFT) gRebootM4Requested = true;
      if(b.pin==BTN_RIGHT) gSpinM4Requested  = true;   // request stress
    }
  }
}

// ===== RTC helpers =====
static void rtcSetInitial(int Y,int M,int D,int h,int m,int s){ struct tm t={0}; t.tm_year=Y-1900; t.tm_mon=M-1; t.tm_mday=D; t.tm_hour=h; t.tm_min=m; t.tm_sec=s; set_time(mktime(&t)); }
static bool rtcIsPlausible(){ time_t now=time(NULL); return (now>1704067200UL)&&(now<2051222400UL); }
static void rtcEnsureValid(){ if(!rtcIsPlausible()){ rtcSetInitial(2025,8,28,10,38,0); } }
static bool okHeldAtBoot(){ pinMode(BTN_OK,INPUT); uint32_t t0=millis(); int lows=0,samples=0; while(millis()-t0<200){ lows+=(digitalRead(BTN_OK)==LOW); samples++; delay(5);} return lows>samples*0.8; }

// ===== Helpers =====
static inline void fmtCpuLines(char *l1, size_t n1, char *l2, size_t n2, int pct, int mhz){
  if (pct < 0) { snprintf(l1,n1,"L: OFF"); snprintf(l2,n2,"F: %dMHz", mhz); }
  else         { snprintf(l1,n1,"L: %d%%", pct); snprintf(l2,n2,"F: %dMHz", mhz); }
}
static void fmtUptime(char *out, size_t n){
  uint32_t ms = millis();
  uint32_t sec = ms / 1000;
  uint32_t days = sec / 86400;  sec %= 86400;
  uint32_t hrs  = sec / 3600;   sec %= 3600;
  uint32_t mins = sec / 60;     sec %= 60;
  if (days > 0) snprintf(out, n, "%lud%02luh", (unsigned long)days, (unsigned long)hrs);
  else          snprintf(out, n, "%02lu:%02lu:%02lu", (unsigned long)hrs, (unsigned long)mins, (unsigned long)sec);
}

// Handshake: wait briefly for M4 to announce
static bool waitForM4Ready(uint32_t timeout_ms) {
  uint32_t t0 = millis();
  while (RPC.available()) { (void)RPC.read(); }
  RPC.println("PING");
  while ((millis() - t0) < timeout_ms) {
    while (RPC.available()) {
      String s = RPC.readStringUntil('\n'); s.trim();
      if (s.startsWith("M4:READY") || s.startsWith("M4:HB") || s.startsWith("M4,")) {
        gM4LastMs = millis();
        return true;
      }
    }
    delay(10);
  }
  return false;
}

// ===== SETUP / LOOP =====
void setup() {
  Serial.begin(115200);
  BoardTemp_begin();

  Breakout.pinMode(PIN_BL, OUTPUT);
  blSet(gBrightness);

  SPI.begin(); gfx->begin(48000000); gfx->invertDisplay(INVERT_COLORS);
  gfx->setTextWrap(false); gfx->setTextSize(2); gfx->setTextColor(COL_TEXT, COL_BG);
  setupButtons(); Breakout.pinMode(PIN_VIBR, OUTPUT); vibOff();

  gfx->fillScreen(COL_BG); computeLayoutPx();
  if (okHeldAtBoot()) { vibrate(120); rtcSetInitial(2025,8,28,10,38,0); } else rtcEnsureValid();

  RPC.begin();
  bootM4(); delay(50);
  if (!waitForM4Ready(2500)) { bootM4(); waitForM4Ready(3000); }

  // UI skeleton
  drawTitleOnce();
  drawClockChips(true); drawFooterOnceThenMaintain();
  drawNiceBox(gBoxTL.x, gBoxTL.y, 12, "Temp", "--");
  char upt[16]; fmtUptime(upt, sizeof(upt)); drawNiceBox(gBoxTR.x, gBoxTR.y, 12, "UPT", upt);
  char a[24], b[24];
  fmtCpuLines(a,sizeof(a), b,sizeof(b), -1, m7MHz()); drawInfoBox(gBoxBL.x, gBoxBL.y, 12, "M7", a, b);
  fmtCpuLines(a,sizeof(a), b,sizeof(b), -1, 0);       drawInfoBox(gBoxBR.x, gBoxBR.y, 12, "M4", a, b);
}

void loop() {
  BoardTemp_poll_1Hz();

  for (uint8_t i = 0; i < 5; ++i) updateButton(buttons[i], i);

  if (gRebootM4Requested) { gRebootM4Requested = false; bootM4(); waitForM4Ready(3000); }
  if (gSpinM4Requested)   {
    gSpinM4Requested = false;
    // Try both protocols so it works with either M4 firmware flavor:
    RPC.println("SPIN,1200");     // newer sketch expects millis duration
    RPC.println("TESTLOAD,1");    // older sketch toggles ON/OFF
    gSpinStopAtMs = millis() + 1500;  // auto-off for TESTLOAD fallback
    Serial.println(F("[M7] Sent SPIN,1200 + TESTLOAD,1"));
  }
  // Stop TESTLOAD after timeout if used:
  if (gSpinStopAtMs && millis() > gSpinStopAtMs) {
    RPC.println("TESTLOAD,0");
    gSpinStopAtMs = 0;
    Serial.println(F("[M7] Sent TESTLOAD,0"));
  }

  // Clock refresh each second
  static int lastSec=-1; time_t now=time(NULL); struct tm *t=localtime(&now);
  if (t->tm_sec!=lastSec){ lastSec=t->tm_sec; drawClockChips(false); }

  // Consume RPC from M4
  while (RPC.available()) {
    String s = RPC.readStringUntil('\n'); s.trim();
    if (s.startsWith("M4,")) {
      int c1 = s.indexOf(','), c2 = s.indexOf(',', c1+1);
      if (c1 > 0 && c2 > c1) {
        gM4LoadPct = constrain(s.substring(c1+1,c2).toInt(), 0, 100);
        gM4MHz     = s.substring(c2+1).toInt();
        gM4LastMs  = millis();
      }
    } else if (s.startsWith("M4:HB") || s.startsWith("M4:READY")) {
      gM4LastMs = millis();
    }
  }

  // If M4 silent >3s, try one auto-reboot
  static bool triedReboot = false;
  if ((millis() - gM4LastMs) > 3000 && !triedReboot) { triedReboot = true; bootM4(); waitForM4Ready(3000); }
  if ((millis() - gM4LastMs) <= 3000) triedReboot = false;

  // UI refresh ~300 ms
  static uint32_t tmo=0;
  if (millis()-tmo > 300) {
    tmo = millis();

    // Temp in °C
    char temp[16] = "--";
    int __c = BoardTemp_getC();
    if (__c != -999) snprintf(temp, sizeof(temp), "%dC", __c);
    drawNiceBox(gBoxTL.x, gBoxTL.y, 12, "Temp", temp);

    // Uptime
    char upt[16]; fmtUptime(upt, sizeof(upt));
    drawNiceBox(gBoxTR.x, gBoxTR.y, 12, "UPT", upt);

    // M7 CPU
    bool ready=false; float load = m7CpuLoadPct(ready);
    int pct = ready ? (int)roundf(load) : -1;
    char a[24], b[24]; fmtCpuLines(a,sizeof(a), b,sizeof(b), pct, m7MHz());
    drawInfoBox(gBoxBL.x, gBoxBL.y, 12, "M7", a, b);

    // M4 CPU (timeout -> OFF if silent)
    if ((millis() - gM4LastMs) > 2000) { gM4LoadPct = -1; }
    fmtCpuLines(a,sizeof(a), b,sizeof(b), gM4LoadPct, gM4MHz);
    drawInfoBox(gBoxBR.x, gBoxBR.y, 12, "M4", a, b);

    // Footer (no flicker)
    drawFooterOnceThenMaintain();
  }

  delay(1); // keep idle thread running for accurate load%
}
