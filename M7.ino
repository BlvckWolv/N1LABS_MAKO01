/*
  Portenta H7 + Breakout + ST7789 (170x320)
  Stable UI with:
   - 5s splash ("N1LABS" + ASCII loading bar, flicker-free)
   - Temp in °C from MAX17262 fuel-gauge (I2C 0x36, Wire1)
   - M7/M4 load via RPC (robust parser)
   - RIGHT tap = stress M4 (SPIN or TESTLOAD)
   - Footer: left "N1LABS" chip, right 8‑bit brightness icon (reflects 0..255 BL)
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

// ---------------- BoardTemp (MAX17262) inline ----------------
namespace BoardTempInline {
  static const uint8_t ADDR = 0x36;
  static const uint8_t REG_TEMP    = 0x08; // signed, 1/256 °C
  static const uint8_t REG_DIETEMP = 0x34; // signed, 1/256 °C
  static TwoWire &W = Wire1;               // Portenta Breakout fuel gauge on Wire1
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

// ---------------- Display / pins ----------------
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

// ---- types & globals ----
struct BtnState { uint8_t pin; bool pressed; const char* name; };
struct BoxPx   { int x; int y; };
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

uint8_t gBrightness = 255;  // full bright so splash is visible

volatile int gM4LoadPct = -1;
volatile int gM4MHz     = 0;
uint32_t     gM4LastMs  = 0;

volatile bool gRebootM4Requested = false;
volatile bool gSpinM4Requested   = false;
static uint32_t gSpinStopAtMs = 0;

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

// ===== Clock chips (Time + Date) =====
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

// ===== Pixel-art brightness icon (8-bit style) =====
// Draws a 5x5 pixel "sun" core with up to 8 rays, scaled by 's'.
// 'level' is 0..255 (maps to 0..8 rays). Drawn with foreground color 'fg' on background 'bg'.
static void drawBrightnessIcon8(int x, int y, uint16_t fg, uint16_t bg, uint8_t level, int s){
  // Clear a safe bounding box (~(5+2)*s square)
  int bw = (5+2)*s, bh = (5+2)*s;
  gfx->fillRect(x, y, bw, bh, COL_BG);

  // Core: 3x3 square centered in 5x5 grid
  for (int dy=1; dy<=3; ++dy){
    for (int dx=1; dx<=3; ++dx){
      gfx->fillRect(x + (dx*s), y + (dy*s), s, s, fg);
    }
  }

  // Map level (0..255) to 0..8 rays
  int rays = (int)((level * 8 + 127) / 255); // rounded
  if (rays < 0) rays = 0; if (rays > 8) rays = 8;

  // Ray order: N, E, S, W, NE, SE, SW, NW
  const int RX[8] = {2,4,2,0,4,4,0,0};
  const int RY[8] = {0,2,4,2,0,4,4,0};

  for (int i=0; i<rays; ++i){
    gfx->fillRect(x + RX[i]*s, y + RY[i]*s, s, s, fg);
  }
}

// Footer BL icon + N1LABS chip
static bool footerInitDone=false; static int lastBLpct=-1;
static void drawFooterInit(){
  const int y = 170 - CHAR_H - 2;
  gfx->fillRect(0, y, 320, CHAR_H+2, COL_BG);
  const char *LOGO="N1LABS"; int textW=(int)strlen(LOGO)*CHAR_W; int padX=5,padY=1;
  int chipW=textW+padX*2, chipH=CHAR_H+padY*2-2, chipX=8, chipY=y-1, radius=2;
  gfx->fillRoundRect(chipX, chipY, chipW, chipH, radius, COL_CHIP);
  gfx->setTextColor(COL_CHIPFG, COL_CHIP); gfx->setTextSize(2); gfx->setCursor(chipX+padX, y); gfx->print(LOGO);
  footerInitDone=true; lastBLpct=-1;
}
static void drawFooterBLIfChanged(){
  const int y = 170 - CHAR_H - 2;
  int pct = (int)gBrightness * 100 / 255;
  if(pct == lastBLpct && footerInitDone) return;
  lastBLpct = pct;

  // Place icon at far right with small padding
  int s = 2;                 // pixel scale
  int iconW = (5+2)*s;       // as used in drawBrightnessIcon8
  int iconH = (5+2)*s;
  int pad = 6;
  int x = 320 - pad - iconW;
  int iy = y + (CHAR_H - iconH)/2; if (iy < y) iy = y;

  // Draw pixel-art sun icon; level uses raw 0..255 brightness
  drawBrightnessIcon8(x, iy, COL_TEXT, COL_BG, gBrightness, s);
}
static void drawFooterOnceThenMaintain(){ if(!footerInitDone) drawFooterInit(); drawFooterBLIfChanged(); }

// ===== Buttons =====
static void setupButtons(){ pinMode(BTN_UP,INPUT); pinMode(BTN_DOWN,INPUT); pinMode(BTN_LEFT,INPUT); pinMode(BTN_RIGHT,INPUT); pinMode(BTN_OK,INPUT); }
static void adjustBrightness(int d){ int v=(int)gBrightness+d; v=constrain(v,0,255); if(v!=gBrightness){ gBrightness=(uint8_t)v; blSet(gBrightness); drawFooterBLIfChanged(); } }
static void updateButton(uint8_t i){
  BtnState &b = buttons[i];
  bool raw=(digitalRead(b.pin)==LOW); uint32_t now=millis();
  if(raw!=b.pressed && (now-lastChange[i])>DEBOUNCE_MS){
    b.pressed=raw; lastChange[i]=now;
    if(b.pressed){
      vibOn(); delay(80); vibOff();
      if(b.pin==BTN_UP)   adjustBrightness(+16);
      if(b.pin==BTN_DOWN) adjustBrightness(-16);
      if(b.pin==BTN_LEFT) gRebootM4Requested = true;
      if(b.pin==BTN_RIGHT) gSpinM4Requested  = true;
    }
  }
}

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

// ===== RPC helper =====
static void handleRPCLine(const String &s){
  if (s.startsWith("M4,")) {
    char buf[64]; s.toCharArray(buf, sizeof(buf));
    int load= -1, mhz = 0;
    if (2 == sscanf(buf, "M4,%d,%d", &load, &mhz)) {
      if (load < 0) load = -1; if (load > 100) load = 100;
      if (mhz >= 1800 && mhz <= 2200) mhz /= 10;  // fix "2004"
      if (mhz < 0 || mhz > 600) mhz = 200;
      gM4LoadPct = load; gM4MHz = mhz; gM4LastMs = millis();
    }
  } else if (s.startsWith("M4:HB") || s.startsWith("M4:READY")) {
    gM4LastMs = millis();
  }
}

// ===== Splash (5s) =====
static void showSplash5s(){
  gfx->fillScreen(COL_BG);
  gfx->setTextColor(COL_TEXT, COL_BG); // text overwrites without clear

  const char *label = "N1LABS";
  const int cw3 = 18, ch3 = 24;
  int lw = (int)strlen(label) * cw3;
  int lx = (320 - lw)/2;
  int ly = (170 - ch3)/2 - 12; // a bit higher to fit bar + text

  gfx->setTextSize(3);
  gfx->setCursor(lx, ly);
  gfx->print(label);

  // Shorter ASCII loading bar under the label (static frame, incremental fill)
  const int cw2 = 12, ch2 = 16;
  const int barChars = 14; // shorter bar
  int bar_px = (barChars + 2) * cw2; // [ + chars + ]
  int x = (320 - bar_px) / 2;
  int y = ly + ch3 + 4; // directly under the logo

  gfx->setTextSize(2);
  // Draw static frame: [              ]
  gfx->setCursor(x, y);
  gfx->print('[');
  for (int i=0;i<barChars;i++) gfx->print(' ');
  gfx->print(']');

  // Fixed-width status text (no flicker): always print same-length string at same x
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

    // Only draw newly filled segments (no full clears)
    if (filled > last_filled){
      for (int i = last_filled; i < filled; ++i){
        gfx->setCursor(x + cw2*(1 + i), y);
        gfx->print('=');
      }
      last_filled = filled;
    }

    // Print fixed-width message; background color overwrites previous without clearing
    if (pct != last_pct){
      char msg[32];
      snprintf(msg, sizeof(msg), "Initializing... %3d%%", pct); // fixed width
      gfx->setCursor(msg_x, msg_y);
      gfx->print(msg);
      last_pct = pct;
    }

    delay(30);
  }

  // Ensure complete
  for (int i = last_filled; i < barChars; ++i){
    gfx->setCursor(x + cw2*(1 + i), y);
    gfx->print('=');
  }
  gfx->setCursor(msg_x, msg_y);
  gfx->print("Initializing... 100%");
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

  // Splash first, guaranteed visible
  showSplash5s();
  gfx->fillScreen(COL_BG);

  computeLayoutPx();

  // UI skeleton
  drawTitleOnce(); drawClockChips(true); drawFooterOnceThenMaintain();
  drawNiceBox(gBoxTL.x, gBoxTL.y, 12, "Temp", "--");
  char upt[16]; fmtUptime(upt, sizeof(upt)); drawNiceBox(gBoxTR.x, gBoxTR.y, 12, "UPT", upt);
  char a[24], b[24];
  fmtCpuLines(a,sizeof(a), b,sizeof(b), -1, m7MHz()); drawInfoBox(gBoxBL.x, gBoxBL.y, 12, "M7", a, b);
  fmtCpuLines(a,sizeof(a), b,sizeof(b), -1, 0);       drawInfoBox(gBoxBR.x, gBoxBR.y, 12, "M4", a, b);

  // Start RPC AFTER UI so UI is not blocked by waits
  RPC.begin();
  bootM4();
}

void loop() {
  // Temp
  BoardTemp_poll_1Hz();

  // Buttons
  for (uint8_t i = 0; i < 5; ++i) updateButton(i);

  if (gRebootM4Requested) { gRebootM4Requested = false; bootM4(); }
  if (gSpinM4Requested)   {
    gSpinM4Requested = false;
    RPC.println("SPIN,1200");     // newer M4
    RPC.println("TESTLOAD,1");    // older M4 fallback
    gSpinStopAtMs = millis() + 1500;
  }
  if (gSpinStopAtMs && millis() > gSpinStopAtMs) { RPC.println("TESTLOAD,0"); gSpinStopAtMs = 0; }

  // RPC pump
  while (RPC.available()) {
    String s = RPC.readStringUntil('\n'); s.trim(); handleRPCLine(s);
  }

  // If M4 silent >3s, mark as off (no blocking waits)
  if ((millis() - gM4LastMs) > 2000) { gM4LoadPct = -1; }

  // Clock refresh each second
  static int lastSec=-1; time_t now=time(NULL); struct tm *t=localtime(&now);
  if (t->tm_sec!=lastSec){ lastSec=t->tm_sec; drawClockChips(false); }

  // UI refresh ~300 ms
  static uint32_t tmo=0;
  if (millis()-tmo > 300) {
    tmo = millis();

    // Temp °C
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

    // M4 CPU
    fmtCpuLines(a,sizeof(a), b,sizeof(b), gM4LoadPct, gM4MHz);
    drawInfoBox(gBoxBR.x, gBoxBR.y, 12, "M4", a, b);

    drawFooterOnceThenMaintain();
  }

  delay(1);
}
