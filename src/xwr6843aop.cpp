#include <Arduino.h>

// Arduino's binary.h defines B0, B1, B2, B3 which clash with Eigen
#undef B0
#undef B1
#undef B2
#undef B3

#include "xwr6843aop.h"
#include <string.h>
#include <math.h>

// ── Radar chirp configuration ────────────────────────────────
#include "../config/icins2021.h"

// Alias the active config so the rest of the file is config-agnostic
// static uint8_t frame_buf[FRAME_MAX];

// ── Radar configuration commands (AWR6843AOP) ────────────────
static const char* const CFG[] = {
  "sensorStop",
  "configDataPort 921600 1",
  "sensorStop",
  "flushCfg",
  "dfeDataOutputMode 1",
  "channelCfg 15 7 0",
  "adcCfg 2 1",
  "adcbufCfg -1 0 1 1 1",
  "profileCfg 0 60 115 7 15 0 0 100 1 64 9142 0 0 158",
  "chirpCfg 0 0 0 0 0 0 0 1",
  "chirpCfg 1 1 0 0 0 0 0 2",
  "chirpCfg 2 2 0 0 0 0 0 4",
  "frameCfg 0 2 128 0 100 1 0",
  "lowPower 0 0",
  "guiMonitor -1 1 1 0 0 0 1",
  "cfarCfg -1 0 2 8 4 3 0 15 1",
  "cfarCfg -1 1 0 8 4 4 1 15 1",
  "multiObjBeamForming -1 1 0.5",
  "clutterRemoval -1 0",
  "calibDcRangeSig -1 0 -5 8 256",
  "extendedMaxVelocity -1 0",
  "lvdsStreamCfg -1 0 0 0",
  "compRangeBiasAndRxChanPhase 0.0 1 0 -1 0 1 0 -1 0 1 0 -1 0 1 0 -1 0 1 0 -1 0 1 0 -1 0",
  "measureRangeBiasAndRxChanPhase 0 1.5 0.2",
  "CQRxSatMonitor 0 3 4 19 0",
  "CQSigImgMonitor 0 31 4",
  "analogMonitor 0 0",
  "aoaFovCfg -1 -90 90 -90 90",
  "cfarFovCfg -1 0 0 10.97",
  "cfarFovCfg -1 1 -3.2 3.20",
  "calibData 0 0 0",
  "sensorStart",
};
static constexpr size_t NUM_CFG = sizeof(CFG) / sizeof(CFG[0]);



// static const char* const* CFG   = RADAR_CFG_ICINS2021;
// static const size_t       NUM_CFG = NUM_CFG_ICINS2021;

// ── UART handles ─────────────────────────────────────────────
static HardwareSerial& CLI  = Serial2;   // RX=pin7,  TX=pin8
static HardwareSerial& DATA = Serial3;   // RX=pin15, TX=pin14

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
  float f;
  memcpy(&f, p, 4);
  return f;
}

// ── Internal helpers ─────────────────────────────────────────

static bool readBytesTimeout(HardwareSerial& s, uint8_t* dst,
                             size_t n, uint32_t timeout_ms) {
  uint32_t t0 = millis();
  size_t got = 0;
  while (got < n && (millis() - t0) < timeout_ms) {
    while (s.available() && got < n) {
      dst[got++] = (uint8_t)s.read();
    }
  }
  return got == n;
}

/// Send a CLI command and wait for "Done" / "Error".
static bool sendCmd(const char* cmd, uint32_t timeout_ms = 2000) {
  Serial.print("[TX] ");
  Serial.println(cmd);

  CLI.print(cmd);
  CLI.print("\r\n");
  CLI.flush();

  uint32_t t0 = millis();
  char buf[512];
  size_t n = 0;

  while (millis() - t0 < timeout_ms) {
    while (CLI.available()) {
      char c = (char)CLI.read();
      if (n < sizeof(buf) - 1) buf[n++] = c;
      buf[n] = '\0';

      if (strstr(buf, "Done") || strstr(buf, "done")) {
        Serial.print("[RX] "); Serial.println(buf);
        return true;
      }
      if (strstr(buf, "Error") || strstr(buf, "error")) {
        Serial.print("[RX] "); Serial.println(buf);
        return false;
      }
    }
  }

  // Timeout — dump what we got
  Serial.print("[RX] TIMEOUT ("); Serial.print(n); Serial.print("B): ");
  for (size_t i = 0; i < n; i++) {
    uint8_t b = (uint8_t)buf[i];
    if (b >= 0x20 && b < 0x7F) Serial.print((char)b);
    else { Serial.print("<0x"); Serial.print(b, HEX); Serial.print(">"); }
  }
  Serial.println();
  return false;
}

/// Send full configuration array over CLI UART.
static bool configure() {
  Serial.println("Sending radar config...");
  for (size_t i = 0; i < NUM_CFG; i++) {
    bool ok = sendCmd(CFG[i]);

    // sensorStop may fail if sensor wasn't running — that's OK
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

/// Read one complete frame into frame_buf.  Returns byte count, 0 on failure.
static size_t readFrame() {
  // 1) Sync to magic word
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
  // 2) Read 32-byte header
  uint8_t hdr[32];
  if (!readBytesTimeout(DATA, hdr, sizeof(hdr), 80)) return 0;

  uint32_t total_len = u32le(hdr + 4);
  if (total_len < 8 + 32 || total_len > FRAME_MAX) return 0;

  memcpy(frame_buf + 0, MAGIC, 8);
  memcpy(frame_buf + 8, hdr, 32);

  // 3) Read remaining payload
  size_t remaining = total_len - 8 - 32;
  if (!readBytesTimeout(DATA, frame_buf + 8 + 32, remaining, 200)) return 0;

  return total_len;
}

/// Walk TLVs and extract type 1 (Detected Points) as rio::RadarDoppler.
/// Each TI detected point is 16 bytes: x(f32) y(f32) z(f32) v(f32).
/// We convert to unit direction u_R = normalize(x,y,z) and vr = v.
static void parseTLVs(const uint8_t* raw, size_t len, RadarFrame& frame) {
  if (len < 8 + 32) return;
  const uint8_t* h = raw + 8;   // header starts after magic

  frame.numObj = u32le(h + 20);
  frame.numTLV = u32le(h + 24);

  const float t = static_cast<float>(millis()) * 1e-3f;

  const uint8_t* ptr = raw + 8 + 32;  // first TLV
  const uint8_t* end = raw + len;

  for (uint32_t t_idx = 0; t_idx < frame.numTLV && ptr + 8 <= end; t_idx++) {
    uint32_t tlvType = u32le(ptr);
    uint32_t tlvLen  = u32le(ptr + 4);
    const uint8_t* tlvData = ptr + 8;
    ptr += 8 + tlvLen;

    if (ptr > end) break;  // malformed

    if (tlvType == 1) {
      // DETECTED_POINTS — 16 bytes each (x, y, z, v as float32)
      uint32_t nPts = tlvLen / 16;
      for (uint32_t i = 0; i < nPts && frame.numMeas < RadarFrame::MAX_POINTS; i++) {
        const uint8_t* p = tlvData + i * 16;
        float x = f32le(p + 0);
        float y = f32le(p + 4);
        float z = f32le(p + 8);
        float v = f32le(p + 12);

        float range = sqrtf(x * x + y * y + z * z);
        if (range < 1e-3f) continue;  // skip zero-range points

        rio::RadarDoppler& d = frame.meas[frame.numMeas++];
        d.t     = t;
        d.u_R   = rio::Vec3(x, y, z) / range;
        d.vr    = v;
        d.sigma = 0.0f;  // will use params.sigma_vr in eskf.correct()
      }
    }
  }
}

// ── Public API ───────────────────────────────────────────────

bool xwr6843aopInit() {
  // Start UARTs early to catch boot banner
  CLI.begin(CLI_BAUD);
  DATA.begin(DATA_BAUD);

  // Drain boot banner (5 s)
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

  // Print boot banner
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
  frame.numObj  = 0;
  frame.numTLV  = 0;
  frame.numMeas = 0;
  frame.valid   = false;

  size_t len = readFrame();
  if (len == 0) return;

  parseTLVs(frame_buf, len, frame);
  frame.valid = true;
}

void xwr6843aopDrainCli() {
  while (CLI.available()) {
    Serial.print((char)CLI.read());
  }
}

void xwr6843aopPrintFrame(const RadarFrame& frame) {
  Serial.print("RADAR  obj=");
  Serial.print(frame.numObj);
  Serial.print("  tlvs=");
  Serial.print(frame.numTLV);
  Serial.print("  meas=");
  Serial.println(frame.numMeas);

  for (uint32_t i = 0; i < frame.numMeas; i++) {
    const rio::RadarDoppler& d = frame.meas[i];
    Serial.print("  [");
    Serial.print(i);
    Serial.print("]  u=(");
    Serial.print(d.u_R.x(), 2);
    Serial.print(", ");
    Serial.print(d.u_R.y(), 2);
    Serial.print(", ");
    Serial.print(d.u_R.z(), 2);
    Serial.print(")  vr=");
    Serial.print(d.vr, 2);
    Serial.println(" m/s");
  }
}
