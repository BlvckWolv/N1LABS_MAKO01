// M4.ino — Portenta H7 (Cortex‑M4) telemetry with REAL utilization + TIME_SYNC trigger
// - Sends:  M4,<load_pct>,<MHz>   every 250 ms (windowed, measured busy time)
// - Heartbeat: M4:HB              every 500 ms
// - RPC: PING -> M4:PONG
//        TESTLOAD,1/0 -> enable/disable synthetic load, ack M4:TESTLOAD,ON/OFF
// - After ~5s from boot, sends TIME_SYNC to M7 so M7 Wi‑Fi syncs RTC, then powers Wi‑Fi off.
//
// Utilization model: busy_us accumulates time consumed in
//   * processCommands()
//   * synthetic busy-wait (if gTestLoad)
//   * sending metrics + heartbeat
// pct = 100 * busy_us / window_us

#include <Arduino.h>
#include <RPC.h>
#include <math.h>

// ---------- config ----------
#define METRICS_WINDOW_MS    250      // reporting window
#define HEARTBEAT_MS         500
#define TESTLOAD_BUSY_USEC   50000    // ~50 ms synthetic load when enabled
#define TIME_SYNC_DELAY_MS   5000     // ask M7 to sync time after this delay

// ---------- state ----------
volatile bool gTestLoad = false;

static uint32_t winStartMs   = 0;
static uint32_t busyUsAccum  = 0;
static uint32_t lastHBMs     = 0;

static bool     gAskedTimeSync = false;
static uint32_t gBootMs        = 0;

static inline int m4MHz(){
  return (int)(SystemCoreClock / 1000000UL);
}

// ---- helpers to account "busy" time ----
struct BusyTimer {
  uint32_t t0;
  BusyTimer() : t0(micros()) {}
  ~BusyTimer() { busyUsAccum += (uint32_t)(micros() - t0); }
};

static void processCommands(){
  BusyTimer _busy; // account time spent handling RPC
  while (RPC.available()){
    String s = RPC.readStringUntil('\n');
    s.trim();
    if (s.length()==0) continue;

    if (s == "PING"){
      RPC.println("M4:PONG");
    } else if (s.startsWith("TESTLOAD,")){
      int v = s.substring(9).toInt();
      gTestLoad = (v != 0);
      RPC.print("M4:TESTLOAD,"); RPC.println(gTestLoad ? "ON" : "OFF");
    } else if (s == "TESTLOAD?"){
      RPC.print("M4:TESTLOAD,"); RPC.println(gTestLoad ? "ON" : "OFF");
    }
  }
}

static void sendMetrics(){
  BusyTimer _busy; // account time spent formatting/sending
  uint32_t nowMs = millis();
  uint32_t winMs = nowMs - winStartMs;
  if (winMs == 0) winMs = METRICS_WINDOW_MS;
  uint32_t winUs = winMs * 1000UL;
  float pctf = (winUs > 0) ? (100.0f * (float)busyUsAccum / (float)winUs) : 0.0f;
  int pct = (int)roundf(constrain(pctf, 0.0f, 100.0f));
  RPC.print("M4,"); RPC.print(pct); RPC.print(","); RPC.println(m4MHz());
  winStartMs  = nowMs;
  busyUsAccum = 0;
}

void setup(){
  RPC.begin();
  RPC.println("M4:READY");

  gBootMs     = millis();
  winStartMs  = gBootMs;
  lastHBMs    = gBootMs;

  // Initial metrics & heartbeat
  sendMetrics();
  RPC.println("M4:HB");
}

void loop(){
  processCommands();

  // Ask M7 to sync time once, a few seconds after boot
  if (!gAskedTimeSync && (millis() - gBootMs) > TIME_SYNC_DELAY_MS){
    gAskedTimeSync = true;
    RPC.println("TIME_SYNC");
  }

  // Synthetic CPU load (if enabled)
  if (gTestLoad){
    BusyTimer _busy; // count this as busy
    const uint32_t t_end = micros() + TESTLOAD_BUSY_USEC;
    while ((int32_t)(micros() - t_end) < 0) { __asm__ __volatile__("nop"); }
  }

  // Periodic metrics
  if ((uint32_t)(millis() - winStartMs) >= METRICS_WINDOW_MS){
    sendMetrics();
  }

  // Heartbeat
  if ((uint32_t)(millis() - lastHBMs) >= HEARTBEAT_MS){
    lastHBMs = millis();
    BusyTimer _busy; // account time to emit HB
    RPC.println("M4:HB");
  }
}

