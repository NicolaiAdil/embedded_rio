#include <Arduino.h>
#include <Wire.h>

// Arduino's binary.h defines B0, B1, B2, B3 which clash with Eigen
#undef B0
#undef B1
#undef B2
#undef B3

#include <rio/rio_eskf.h>

// ──────────────────────────────────────────────────────────────
// RADAR UART (Teensy 4.1)
//   CLI  UART: 115200 (commands + "Done")
//   DATA UART: 921600 (binary stream)
// Wiring (TTL 3.3V):
//   Radar GND    <-> Teensy GND
//   Radar CLI_TX ->  Teensy Serial2 RX (pin 7)
//   Radar CLI_RX <-  Teensy Serial2 TX (pin 8)
//   Radar DATA_TX->  Teensy Serial3 RX (pin 15)
//   Radar DATA_RX<-  Teensy Serial3 TX (pin 14)   (optional, usually unused)
// ──────────────────────────────────────────────────────────────
HardwareSerial& RADAR_CLI  = Serial2;   // RX=pin7,  TX=pin8
HardwareSerial& RADAR_DATA = Serial3;   // RX=pin15, TX=pin14

static constexpr uint32_t RADAR_CLI_BAUD  = 115200;
static constexpr uint32_t RADAR_DATA_BAUD = 921600;

static constexpr uint8_t  MAGIC[8] = {2,1,4,3,6,5,8,7};
static constexpr size_t   RADAR_FRAME_MAX = 4096;   // increase if you output more TLVs
static uint8_t            radar_frame[RADAR_FRAME_MAX];

// ──────────────────────────────────────────────────────────────
// BMI085 I2C addresses (SDO1/SDO2 → VDDIO)
// ──────────────────────────────────────────────────────────────
static constexpr uint8_t ACC_ADDR = 0x19;
static constexpr uint8_t GYR_ADDR = 0x69;

// ──────────────────────────────────────────────────────────────
// BMI085 register addresses
// ──────────────────────────────────────────────────────────────
static constexpr uint8_t REG_ACC_CHIP_ID   = 0x00;
static constexpr uint8_t REG_ACC_X_LSB     = 0x12;
static constexpr uint8_t REG_ACC_RANGE     = 0x41;  // 0x00=±2g 0x01=±4g 0x02=±8g 0x03=±16g
static constexpr uint8_t REG_ACC_PWR_CONF  = 0x7C;
static constexpr uint8_t REG_ACC_PWR_CTRL  = 0x7D;

static constexpr uint8_t REG_GYR_CHIP_ID   = 0x00;
static constexpr uint8_t REG_GYR_X_LSB     = 0x02;
static constexpr uint8_t REG_GYR_RANGE     = 0x0F;  // 0x00=±2000 … 0x04=±125 °/s

// ──────────────────────────────────────────────────────────────
// Sensor configuration & scale factors
// ──────────────────────────────────────────────────────────────
static constexpr uint8_t ACC_RANGE_REG_VAL = 0x02;   // ±8g
static constexpr float   ACC_RANGE_G       = 8.0f;
static constexpr float   ACC_SCALE = ACC_RANGE_G * 9.80665f / 32768.0f;

static constexpr uint8_t GYR_RANGE_REG_VAL = 0x00;   // ±2000°/s
static constexpr float   GYR_SENSITIVITY   = 16.384f; // LSB/°/s at ±2000°/s
static constexpr float   GYR_SCALE = (1.0f / GYR_SENSITIVITY) * (M_PI / 180.0f);

// ──────────────────────────────────────────────────────────────
// I2C helpers (with error detection)
// ──────────────────────────────────────────────────────────────
static bool i2c_ok = true;  // cleared on any I2C error

static bool i2cWriteReg(uint8_t dev, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(dev);
  Wire.write(reg);
  Wire.write(val);
  uint8_t err = Wire.endTransmission();
  if (err != 0) { i2c_ok = false; return false; }
  return true;
}

static bool i2cReadReg(uint8_t dev, uint8_t reg, uint8_t& out) {
  Wire.beginTransmission(dev);
  Wire.write(reg);
  uint8_t err = Wire.endTransmission(false);
  if (err != 0) { i2c_ok = false; out = 0; return false; }
  if (Wire.requestFrom(dev, (uint8_t)1) != 1) { i2c_ok = false; out = 0; return false; }
  out = Wire.read();
  return true;
}

static bool i2cReadBurst(uint8_t dev, uint8_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(dev);
  Wire.write(reg);
  uint8_t err = Wire.endTransmission(false);
  if (err != 0) { i2c_ok = false; memset(buf, 0, len); return false; }
  if (Wire.requestFrom(dev, len) != len) { i2c_ok = false; memset(buf, 0, len); return false; }
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

// ──────────────────────────────────────────────────────────────
// ESKF filter
// ──────────────────────────────────────────────────────────────
static rio::RioEskf eskf;

static rio::Params makeParams() {
  rio::Params p;

  p.g_W = rio::Vec3(0.0f, 0.0f, -9.80665f);

  // IMU noise (continuous-time spectral densities)
  p.sigma_acc = 0.00132435f;
  p.sigma_gyr = 0.01f;
  p.sigma_ba  = 0.01f;
  p.sigma_bg  = 0.001f;

  p.tau_ba = 1000.0f;
  p.tau_bg =  500.0f;

  p.min_dt = 1e-4f;
  p.max_dt = 0.1f;

  p.q_IR = rio::Quat::Identity();
  p.p_IR = rio::Vec3::Zero();
  p.sigma_vr      = 0.038f;
  p.gating_enable = true;
  p.gate_nsigma   = 3.0f;
  p.vr_sign       = 1.0f;

  return p;
}

static constexpr float P0_diag[21] = {
  1e-6f, 1e-6f, 1e-6f,
  1e-2f, 1e-2f, 1e-2f,
  1e-4f, 1e-4f, 1e-4f,
  1.1e-2f, 1.1e-2f, 1e-12f,
  1e-8f, 1e-8f, 1e-8f,
  4e-6f, 4e-6f, 4e-6f,
  7.6e-5f, 7.6e-5f, 7.6e-5f
};

static bool  att_initialized  = false;
static bool  time_initialized = false;
static float last_t           = 0.0f;
static rio::ImuSample last_imu{};

// ──────────────────────────────────────────────────────────────
// RADAR helpers (minimal)
// ──────────────────────────────────────────────────────────────
static inline uint32_t u32le(const uint8_t* p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static bool readBytesTimeout(HardwareSerial& s, uint8_t* dst, size_t n, uint32_t timeout_ms) {
  uint32_t t0 = millis();
  size_t got = 0;
  while (got < n && (millis() - t0) < timeout_ms) {
    while (s.available() && got < n) {
      dst[got++] = (uint8_t)s.read();
    }
  }
  return got == n;
}

static bool radarSendCmd(const char* cmd, uint32_t timeout_ms = 2000) {
  Serial.print("[TX] ");
  Serial.println(cmd);

  RADAR_CLI.print(cmd);
  RADAR_CLI.print("\r\n");
  RADAR_CLI.flush();  // ensure TX completes

  uint32_t t0 = millis();
  char buf[512];
  size_t n = 0;

  while (millis() - t0 < timeout_ms) {
    while (RADAR_CLI.available()) {
      char c = (char)RADAR_CLI.read();
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
  // Print whatever we got, showing non-printable as hex
  Serial.print("[RX] TIMEOUT ("); Serial.print(n); Serial.print("B): ");
  for (size_t i = 0; i < n; i++) {
    uint8_t b = (uint8_t)buf[i];
    if (b >= 0x20 && b < 0x7F) Serial.print((char)b);
    else { Serial.print("<0x"); Serial.print(b, HEX); Serial.print(">"); }
  }
  Serial.println();
  return false;
}

// Minimal config:
// - stop sensor
// - set data port speed to match RADAR_DATA_BAUD
// - (you must add your profileCfg/channelCfg/frameCfg/guiMonitor/etc here)
// - start sensor (no external trigger)
static const char* RADAR_CFG[] = {
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

static constexpr size_t NUM_CFG = sizeof(RADAR_CFG) / sizeof(RADAR_CFG[0]);

static bool radarConfigure() {
  Serial.println("Sending radar config...");
  for (size_t i = 0; i < NUM_CFG; i++) {
    bool ok = radarSendCmd(RADAR_CFG[i]);

    // sensorStop may fail if sensor wasn't running — OK
    if (!ok && strcmp(RADAR_CFG[i], "sensorStop") == 0) {
      Serial.println("  (sensorStop fail is OK)");
      delay(50);
      while (RADAR_CLI.available()) RADAR_CLI.read();
      continue;
    }
    if (!ok) {
      Serial.print(">>> FAILED at: "); Serial.println(RADAR_CFG[i]);
      return false;
    }
    delay(20);
  }
  Serial.println("All config commands accepted!");
  return true;
}

// Reads one complete frame into radar_frame, returns length in out_len.
// Resyncs using magic word.
static bool radarReadFrame(size_t& out_len) {
  out_len = 0;

  // 1) find magic word
  uint8_t win[8] = {0};
  uint32_t t0 = millis();
  while (millis() - t0 < 300) {
    while (RADAR_DATA.available()) {
      uint8_t b = (uint8_t)RADAR_DATA.read();
      memmove(win, win + 1, 7);
      win[7] = b;
      if (memcmp(win, MAGIC, 8) == 0) goto magic_found;
    }
  }
  return false;

magic_found:
  // 2) read 32-byte header (matches your PX4 driver)
  uint8_t hdr[32];
  if (!readBytesTimeout(RADAR_DATA, hdr, sizeof(hdr), 80)) return false;

  // totalPacketLen (bytes) includes magic word in typical TI demos
  uint32_t total_len = u32le(hdr + 4);
  if (total_len < 8 + 32) return false;
  if (total_len > RADAR_FRAME_MAX) return false;

  memcpy(radar_frame + 0, MAGIC, 8);
  memcpy(radar_frame + 8, hdr, 32);

  size_t remaining = total_len - 8 - 32;
  if (!readBytesTimeout(RADAR_DATA, radar_frame + 8 + 32, remaining, 200)) return false;

  out_len = total_len;
  return true;
}

// Minimal “sanity print”: shows number of objects + TLVs.
// You can replace with full point parsing later.
static void radarDebugPrintFrame(const uint8_t* frame, size_t len) {
  if (len < 8 + 32) return;
  const uint8_t* h = frame + 8;

  uint32_t totalLen = u32le(h + 4);
  uint32_t numObj   = u32le(h + 20);
  uint32_t numTLV   = u32le(h + 24);

  Serial.print("RADAR\tlen=");
  Serial.print((unsigned)len);
  Serial.print("\ttotal=");
  Serial.print((unsigned)totalLen);
  Serial.print("\tobj=");
  Serial.print((unsigned)numObj);
  Serial.print("\ttlvs=");
  Serial.println((unsigned)numTLV);
}

// ──────────────────────────────────────────────────────────────
// IMU
// ──────────────────────────────────────────────────────────────
static bool initBMI085() {
  uint8_t id = 0;

  if (!i2cWriteReg(ACC_ADDR, REG_ACC_PWR_CTRL, 0x04)) return false;
  delay(5);
  if (!i2cWriteReg(ACC_ADDR, REG_ACC_PWR_CONF, 0x00)) return false;
  delay(50);

  if (!i2cReadReg(ACC_ADDR, REG_ACC_CHIP_ID, id)) return false;
  Serial.print("Accel chip ID: 0x"); Serial.println(id, HEX);
  if (id != 0x1F) return false;

  if (!i2cWriteReg(ACC_ADDR, REG_ACC_RANGE, ACC_RANGE_REG_VAL)) return false;
  delay(2);

  if (!i2cReadReg(GYR_ADDR, REG_GYR_CHIP_ID, id)) return false;
  Serial.print("Gyro  chip ID: 0x"); Serial.println(id, HEX);
  if (id != 0x0F) return false;

  if (!i2cWriteReg(GYR_ADDR, REG_GYR_RANGE, GYR_RANGE_REG_VAL)) return false;
  delay(2);

  i2c_ok = true;
  return true;
}

static void recoverI2C() {
  Serial.println("I2C error — recovering...");

  Wire.end();
  pinMode(19, OUTPUT); // SCL
  for (int i = 0; i < 16; i++) {
    digitalWriteFast(19, LOW);  delayMicroseconds(5);
    digitalWriteFast(19, HIGH); delayMicroseconds(5);
  }
  pinMode(19, INPUT);

  Wire.begin();
  Wire.setClock(400000);
  delay(10);

  if (initBMI085()) Serial.println("Recovery OK");
  else              Serial.println("Recovery FAILED");
}

static bool readIMU(rio::Vec3& acc, rio::Vec3& gyr) {
  uint8_t buf[6];

  if (!i2cReadBurst(ACC_ADDR, REG_ACC_X_LSB, buf, 6)) return false;
  acc.x() = (int16_t)(buf[0] | (buf[1] << 8)) * ACC_SCALE;
  acc.y() = (int16_t)(buf[2] | (buf[3] << 8)) * ACC_SCALE;
  acc.z() = (int16_t)(buf[4] | (buf[5] << 8)) * ACC_SCALE;

  if (!i2cReadBurst(GYR_ADDR, REG_GYR_X_LSB, buf, 6)) return false;
  gyr.x() = (int16_t)(buf[0] | (buf[1] << 8)) * GYR_SCALE;
  gyr.y() = (int16_t)(buf[2] | (buf[3] << 8)) * GYR_SCALE;
  gyr.z() = (int16_t)(buf[4] | (buf[5] << 8)) * GYR_SCALE;

  return true;
}

static void processImu(const rio::Vec3& f_b, const rio::Vec3& w_b) {
  const float t = static_cast<float>(millis()) * 1e-3f;

  if (!time_initialized) {
    last_t = t;
    time_initialized = true;
    return;
  }

  if (!att_initialized) {
    if (!eskf.initAttitudeFromGravity(f_b, P0_diag, t)) return;
    att_initialized = true;
    last_t = t;
    Serial.println("Attitude initialized from gravity");
    return;
  }

  const float dt = t - last_t;
  last_t = t;

  rio::ImuSample s;
  s.t   = t;
  s.acc = f_b;
  s.gyr = w_b;
  last_imu = s;

  eskf.predict(s, dt);
  eskf.insPropagation(s, dt);

  // Later: when you parse Doppler/points, call eskf.correct(...)
  eskf.advancePriorToPosterior();
}

// ──────────────────────────────────────────────────────────────
// Setup / loop
// ──────────────────────────────────────────────────────────────
static void scanI2C() {
  Serial.println("Scanning I2C bus...");
  int n = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.print("  0x"); Serial.println(a, HEX);
      n++;
    }
  }
  Serial.print("  Total: "); Serial.print(n); Serial.println(" device(s)");
  Serial.println("──────────────────────────────────");
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Wire.begin();
  Wire.setClock(400000);
  scanI2C();
  if (!initBMI085()) {
    Serial.println("BMI085 init failed! Check wiring.");
    while (1) delay(100);
  }

  // ESKF
  rio::Params p = makeParams();
  eskf.setParams(p);
  rio::NominalState x0;
  x0.p_IR = p.p_IR;
  x0.q_IR = p.q_IR;
  eskf.reset(x0, P0_diag, 0.0f);

  // Start radar UARTs EARLY so we catch boot messages
  RADAR_CLI.begin(RADAR_CLI_BAUD);
  RADAR_DATA.begin(RADAR_DATA_BAUD);

  // Collect boot banner bytes while waiting (buffer is only 64 bytes,
  // so we drain continuously to avoid overflow)
  Serial.println("Waiting 5s for radar boot...");
  char bootBuf[1024];
  int bc = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < 5000) {
    while (RADAR_CLI.available() && bc < (int)sizeof(bootBuf) - 1) {
      bootBuf[bc++] = (char)RADAR_CLI.read();
    }
    // Also drain DATA to prevent overflow
    while (RADAR_DATA.available()) RADAR_DATA.read();
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

  // Configure radar (don't halt on failure — continue with IMU)
  bool radar_ok = radarConfigure();
  if (!radar_ok) {
    Serial.println("Radar config failed — continuing with IMU only.");
  } else {
    Serial.println("Radar configured OK!");
  }
  Serial.println("RIO ESKF ready");
}

void loop() {
  // --- IMU ---
  rio::Vec3 acc, gyr;
  if (!readIMU(acc, gyr)) {
    recoverI2C();
    return;
  }
  processImu(acc, gyr);

  // --- Forward any CLI bytes to Serial monitor ---
  while (RADAR_CLI.available()) {
    char c = (char)RADAR_CLI.read();
    Serial.print(c);
  }

  // --- RADAR frame read ---
  size_t frame_len = 0;
  if (radarReadFrame(frame_len)) {
    radarDebugPrintFrame(radar_frame, frame_len);
  }
}