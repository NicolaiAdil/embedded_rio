#include <Arduino.h>

#undef B0
#undef B1
#undef B2
#undef B3

#include "xwr6843aop.h"
#include <string.h>
#include <math.h>

#include "../config/icins2021.h"

// ── Radar configuration commands (AWR6843AOP) ────────────────
static const char* const CFG[] = {
  "sensorStop",
  "configDataPort 921600 1",  // keep: sets DATA UART to 921600 baud
  "sensorStop",
  "flushCfg",
  "dfeDataOutputMode 1",
  "channelCfg 15 7 0",
  "adcCfg 2 1",
  "adcbufCfg -1 0 1 1 1",
  "profileCfg 0 60 133 7 40 0 0 100 1 128 8000 0 0 158",
  "chirpCfg 0 0 0 0 0 0 0 1",
  "chirpCfg 1 1 0 0 0 0 0 2",
  "chirpCfg 2 2 0 0 0 0 0 4",
  "frameCfg 0 2 32 0 40 1 0",  // triggerSelect=1 (software): Teensy has no HW trigger GPIO
  "lowPower 0 0",
  "guiMonitor -1 2 0 0 0 0 0",
  "cfarCfg -1 0 2 8 4 3 0 8 0",
  "cfarCfg -1 1 0 4 2 3 1 20 1",
  "multiObjBeamForming -1 1 0.5",
  "clutterRemoval -1 0",
  "calibDcRangeSig -1 0 -5 8 256",
  "extendedMaxVelocity -1 0",
  "lvdsStreamCfg -1 0 0 0",
  "compRangeBiasAndRxChanPhase 0.0547942 -0.21072 -0.70407 0.08740 0.69629 -0.13940 -0.74902 0.14636 0.69608 -0.37207 -0.89261 0.17529 0.88217 -0.29736 -0.90204 0.31693 0.84238 -0.59573 -0.77765 0.42157 0.84186 -0.52802 -0.84924 0.51184 0.77029",
  "measureRangeBiasAndRxChanPhase 0 1.5 0.2",
  "CQRxSatMonitor 0 3 4 99 0",
  "CQSigImgMonitor 0 127 4",
  "analogMonitor 0 0",
  "aoaFovCfg -1 -60 60 -60 60",
  "cfarFovCfg -1 0 0.1 9.60",
  "cfarFovCfg -1 1 -2.4 2.40",
  "calibData 0 0 0",
  "sensorStart",
};
static constexpr size_t NUM_CFG = sizeof(CFG) / sizeof(CFG[0]);

// ── UART handles ─────────────────────────────────────────────
static HardwareSerial& CLI  = Serial4;
static HardwareSerial& DATA = Serial3;

static constexpr uint32_t CLI_BAUD  = 115200;
static constexpr uint32_t DATA_BAUD = 921600;

// ── TI frame magic word ──────────────────────────────────────
static constexpr uint8_t MAGIC[8] = {2,1,4,3,6,5,8,7};

// ── Frame buffer ─────────────────────────────────────────────
static constexpr size_t FRAME_MAX = 4096;
static uint8_t frame_buf[FRAME_MAX];

// ── Little-endian helpers ────────────────────────────────────
static inline uint32_t u32le(const uint8_t* p) {
  return (uint32_t)p[0]
       | ((uint32_t)p[1] << 8)
       | ((uint32_t)p[2] << 16)
       | ((uint32_t)p[3] << 24);
}

static inline float f32le(const uint8_t* p) {
  float f; memcpy(&f, p, 4); return f;
}

// ── Internal helpers ─────────────────────────────────────────

static bool readBytesTimeout(HardwareSerial& s, uint8_t* dst,
                              size_t n, uint32_t timeout_ms) {
  uint32_t t0 = millis();
  size_t got = 0;
  while (got < n && (millis() - t0) < timeout_ms)
    while (s.available() && got < n)
      dst[got++] = (uint8_t)s.read();
  return got == n;
}

static bool sendCmd(const char* cmd, uint32_t timeout_ms = 2000) {
  Serial.print("[TX] "); Serial.println(cmd);
  CLI.print(cmd); CLI.print("\r\n"); CLI.flush();

  uint32_t t0 = millis();
  char buf[512];
  size_t n = 0;

  while (millis() - t0 < timeout_ms) {
    while (CLI.available()) {
      char c = (char)CLI.read();
      if (n < sizeof(buf) - 1) buf[n++] = c;
      buf[n] = '\0';
      if (strstr(buf, "Done") || strstr(buf, "done")) {
        Serial.print("[RX] "); Serial.println(buf); return true;
      }
      if (strstr(buf, "Error") || strstr(buf, "error")) {
        Serial.print("[RX] "); Serial.println(buf); return false;
      }
    }
  }
  Serial.print("[RX] TIMEOUT ("); Serial.print(n); Serial.print("B): ");
  for (size_t i = 0; i < n; i++) {
    uint8_t b = (uint8_t)buf[i];
    if (b >= 0x20 && b < 0x7F) Serial.print((char)b);
    else { Serial.print("<0x"); Serial.print(b, HEX); Serial.print(">"); }
  }
  Serial.println();
  return false;
}

static bool configure() {
  Serial.println("Sending radar config...");
  for (size_t i = 0; i < NUM_CFG; i++) {
    bool ok = sendCmd(CFG[i]);
    if (!ok && strcmp(CFG[i], "sensorStop") == 0) {
      Serial.println("  (sensorStop fail is OK)");
      delay(50);
      while (CLI.available()) CLI.read();
      continue;
    }
    if (!ok) {
      Serial.print(">>> FAILED at: "); Serial.println(CFG[i]);
      return false;
    }
    delay(20);
  }
  Serial.println("All config commands accepted!");
  return true;
}

static size_t readFrame() {
  uint8_t win[8] = {0};
  uint32_t t0 = millis();
  while (millis() - t0 < 300) {
    while (DATA.available()) {
      memmove(win, win + 1, 7);
      win[7] = (uint8_t)DATA.read();
      if (memcmp(win, MAGIC, 8) == 0) goto magic_found;
    }
  }
  return 0;

magic_found:
  uint8_t hdr[32];
  if (!readBytesTimeout(DATA, hdr, sizeof(hdr), 80)) return 0;

  uint32_t total_len = u32le(hdr + 4);
  if (total_len < 8 + 32 || total_len > FRAME_MAX) return 0;

  memcpy(frame_buf + 0, MAGIC, 8);
  memcpy(frame_buf + 8, hdr, 32);

  size_t remaining = total_len - 8 - 32;
  if (!readBytesTimeout(DATA, frame_buf + 8 + 32, remaining, 200)) return 0;
  return total_len;
}

/// Walk TLVs and store every detected point verbatim.
/// No filtering, no normalisation — just byte extraction.
static void parseTLVs(const uint8_t* buf, size_t len, RadarFrame& frame) {
  if (len < 8 + 32) return;

  const uint8_t* h = buf + 8;
  frame.numObj = u32le(h + 20);
  frame.numTLV = u32le(h + 24);

  const uint8_t* ptr = buf + 8 + 32;
  const uint8_t* end = buf + len;

  for (uint32_t ti = 0; ti < frame.numTLV && ptr + 8 <= end; ti++) {
    uint32_t tlvType = u32le(ptr);
    uint32_t tlvLen  = u32le(ptr + 4);
    const uint8_t* tlvData = ptr + 8;
    ptr += 8 + tlvLen;
    if (ptr > end) break;

    if (tlvType == 1) {
      // DETECTED_POINTS: 16 bytes each — x(f32) y(f32) z(f32) v(f32)
      // TI TLV native frame: x=lateral(right), y=range(forward), z=elevation(up).
      uint32_t nPts = tlvLen / 16;
      for (uint32_t i = 0; i < nPts && frame.numRaw < RadarFrame::MAX_POINTS; i++) {
        const uint8_t* p = tlvData + i * 16;
        const float raw_x = f32le(p + 0);
        const float raw_y = f32le(p + 4);
        const float raw_z = f32le(p + 8);
        RadarPoint& rp = frame.raw[frame.numRaw++];
        rp.x  = raw_x;
        rp.y  = raw_y;
        rp.z  = raw_z;
        rp.vr = f32le(p + 12);
      }
    }
  }
}

// ── Public API ───────────────────────────────────────────────

bool xwr6843aopInit() {
  CLI.begin(CLI_BAUD);
  DATA.begin(DATA_BAUD);

  Serial.println("Waiting 5s for radar boot...");
  char bootBuf[1024];
  int bc = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < 5000) {
    while (CLI.available() && bc < (int)sizeof(bootBuf) - 1)
      bootBuf[bc++] = (char)CLI.read();
    while (DATA.available()) DATA.read();
  }
  bootBuf[bc] = '\0';

  Serial.print("CLI boot bytes ("); Serial.print(bc); Serial.print("): ");
  if (bc == 0) {
    Serial.println("(none)");
  } else {
    for (int i = 0; i < bc; i++) {
      uint8_t b = (uint8_t)bootBuf[i];
      if (b >= 0x20 && b < 0x7F) Serial.print((char)b);
      else { Serial.print("<0x"); Serial.print(b, HEX); Serial.print(">"); }
    }
    Serial.println();
  }
  Serial.println("──────────────────────────────────");

  return configure();
}

void xwr6843aopUpdate(RadarFrame& frame) {
  frame.numObj = 0;
  frame.numTLV = 0;
  frame.numRaw = 0;
  frame.valid  = false;

  size_t len = readFrame();
  if (len == 0) return;

  parseTLVs(frame_buf, len, frame);
  frame.valid = true;
}

void xwr6843aopDrainCli() {
  while (CLI.available()) Serial.print((char)CLI.read());
}

void xwr6843aopPrintRaw(const RadarFrame& frame) {
  Serial.print("RADAR RAW  obj="); Serial.print(frame.numObj);
  Serial.print("  tlvs=");         Serial.print(frame.numTLV);
  Serial.print("  raw=");          Serial.println(frame.numRaw);

  for (uint32_t i = 0; i < frame.numRaw; i++) {
    const RadarPoint& p = frame.raw[i];
    Serial.print("  ["); Serial.print(i); Serial.print("]");
    Serial.print("  x=");  Serial.print(p.x,  3);
    Serial.print("  y=");  Serial.print(p.y,  3);
    Serial.print("  z=");  Serial.print(p.z,  3);
    Serial.print("  r=");  Serial.print(sqrtf(p.x*p.x + p.y*p.y + p.z*p.z), 3);
    Serial.print("  vr="); Serial.print(p.vr, 3);
    Serial.println(" m/s");
  }
}