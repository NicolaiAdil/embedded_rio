#include <Arduino.h>

// Arduino's binary.h defines B0, B1, B2, B3 etc. which clash with Eigen internals
#undef B0
#undef B1
#undef B2
#undef B3

#include <Adafruit_BNO08x.h>
#include <rio/rio_eskf.h>

// ──────────────────────────────────────────────────────────────
// Hardware config
// ──────────────────────────────────────────────────────────────
#define BNO08X_RESET -1

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

// ──────────────────────────────────────────────────────────────
// ESKF filter
// ──────────────────────────────────────────────────────────────
static rio::RioEskf eskf;

// ──────────────────────────────────────────────────────────────
// Filter tuning — adjust these to match your sensor
// ──────────────────────────────────────────────────────────────
static rio::Params makeParams() {
  rio::Params p;

  p.g_W = rio::Vec3(0.0f, 0.0f, -9.80665f);

  // IMU noise (continuous-time spectral densities)
  p.sigma_acc = 0.10f;   // accelerometer white noise  [m/s²/√Hz]
  p.sigma_gyr = 0.01f;   // gyroscope white noise      [rad/s/√Hz]
  p.sigma_ba  = 0.01f;   // accel bias random-walk      [m/s³/√Hz]
  p.sigma_bg  = 0.001f;  // gyro  bias random-walk      [rad/s²/√Hz]

  // Bias time constants (negative = no Gauss–Markov decay)
  p.tau_ba = 1000.0f;    // [s]
  p.tau_bg =  500.0f;    // [s]

  // dt guard rails
  p.min_dt = 1e-4f;
  p.max_dt = 0.1f;

  // ── Radar extrinsics (placeholder – fill in when radar is mounted) ──
  p.q_IR = rio::Quat::Identity();
  p.p_IR = rio::Vec3::Zero();
  p.sigma_vr     = 0.038f;
  p.gating_enable = true;
  p.gate_nsigma   = 3.0f;
  p.vr_sign       = 1.0f;

  return p;
}

// Initial covariance diagonal (21-state)
//   [ δp(3)  δv(3)  δb_a(3)  δθ(3)  δb_g(3)  δp_IR(3)  δθ_IR(3) ]
static constexpr float P0_diag[21] = {
  1e-6f, 1e-6f, 1e-6f,       // position  [m²]
  1e-2f, 1e-2f, 1e-2f,       // velocity  [m²/s²]
  1e-4f, 1e-4f, 1e-4f,       // accel bias [m²/s⁴]
  1.1e-2f, 1.1e-2f, 1e-12f,  // attitude  [rad²]  (~6° roll/pitch, ~0 yaw)
  1e-8f, 1e-8f, 1e-8f,       // gyro bias [rad²/s²]
  4e-6f, 4e-6f, 4e-6f,       // radar pos [m²]
  7.6e-5f, 7.6e-5f, 7.6e-5f  // radar att [rad²]  (~0.5°)
};

// ──────────────────────────────────────────────────────────────
// State
// ──────────────────────────────────────────────────────────────
static bool att_initialized  = false;
static bool time_initialized = false;
static float last_t          = 0.0f;

// Latest IMU sample (kept for potential radar correction)
static rio::ImuSample last_imu{};

// ──────────────────────────────────────────────────────────────
// BNO08x report setup — only accel + gyro
// ──────────────────────────────────────────────────────────────
void setReports() {
  if (!bno08x.enableReport(SH2_ACCELEROMETER)) {
    Serial.println("Could not enable accelerometer");
  }
  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED)) {
    Serial.println("Could not enable gyroscope");
  }
}

// ──────────────────────────────────────────────────────────────
// Print the current ESKF state over Serial
// ──────────────────────────────────────────────────────────────
static void publishState(float t) {
  const auto& x = eskf.getState();

  // Compact tab-separated line:
  //   T  px py pz  vx vy vz  qw qx qy qz
  Serial.print("S\t");
  Serial.print(t, 4);          Serial.print('\t');

  Serial.print(x.p_WI.x(), 4); Serial.print('\t');
  Serial.print(x.p_WI.y(), 4); Serial.print('\t');
  Serial.print(x.p_WI.z(), 4); Serial.print('\t');

  Serial.print(x.v_WI.x(), 4); Serial.print('\t');
  Serial.print(x.v_WI.y(), 4); Serial.print('\t');
  Serial.print(x.v_WI.z(), 4); Serial.print('\t');

  Serial.print(x.q_WI.w(), 6); Serial.print('\t');
  Serial.print(x.q_WI.x(), 6); Serial.print('\t');
  Serial.print(x.q_WI.y(), 6); Serial.print('\t');
  Serial.println(x.q_WI.z(), 6);
}

// ──────────────────────────────────────────────────────────────
// Arduino setup
// ──────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  // ── I2C bus scan — find all devices on the bus ──
  Wire.begin();
  Serial.println("Scanning I2C bus...");
  int nDevices = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      Serial.print("  Found device at 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
      nDevices++;
    }
  }
  if (nDevices == 0) {
    Serial.println("  No I2C devices found! Check wiring and PS pin (must be VDDIO for I2C).");
  } else {
    Serial.print("  Total: ");
    Serial.print(nDevices);
    Serial.println(" device(s)");
  }
  Serial.println("──────────────────────────────────");

  // ── BMI085 accelerometer init (0x19) ──
  // Accel starts in suspend mode — must be woken up
  // Write 0x04 to ACC_PWR_CTRL (0x7D) to enable the accelerometer
  Wire.beginTransmission(0x19);
  Wire.write(0x7D);
  Wire.write(0x04);
  Wire.endTransmission();
  delay(5); // must wait >= 5ms after power-on per datasheet

  // Write 0x00 to ACC_PWR_CONF (0x7C) to set active mode
  Wire.beginTransmission(0x19);
  Wire.write(0x7C);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(50); // wait for accel to fully wake up

  // Read accel chip ID (register 0x00, should be 0x1F for BMI085)
  Wire.beginTransmission(0x19);
  Wire.write(0x00);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)0x19, (uint8_t)1);
  uint8_t chipId = Wire.read();
  Serial.print("Accel chip ID: 0x");
  Serial.println(chipId, HEX);

  // Read current accel range register (0x41), default should be 0x00 = ±2g for BMI085
  Wire.beginTransmission(0x19);
  Wire.write(0x41);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)0x19, (uint8_t)1);
  uint8_t rangeReg = Wire.read();
  Serial.print("Accel range reg (0x41): 0x");
  Serial.println(rangeReg, HEX);

  // Explicitly set range to ±8g (0x02)
  Wire.beginTransmission(0x19);
  Wire.write(0x41);
  Wire.write(0x02); // BMI085_ACCEL_RANGE_8G
  Wire.endTransmission();
  delay(2);

  // Read back to confirm
  Wire.beginTransmission(0x19);
  Wire.write(0x41);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)0x19, (uint8_t)1);
  rangeReg = Wire.read();
  Serial.print("Accel range after set: 0x");
  Serial.println(rangeReg, HEX);

  // Print raw accel data once to help debug scaling
  Wire.beginTransmission(0x19);
  Wire.write(0x12);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)0x19, (uint8_t)6);
  uint8_t dbg[6];
  for (int i = 0; i < 6; i++) dbg[i] = Wire.read();
  int16_t dbg_z = (int16_t)(dbg[4] | (dbg[5] << 8));
  Serial.print("Raw Z sample: ");
  Serial.println(dbg_z);
  Serial.print("  Expected for 1g @ ±4g: ~8192,  @ ±2g: ~16384,  @ ±3g(BMI088): ~10923");
  Serial.println();

  // ── BMI085 gyroscope init (0x69) ──
  // Gyro boots into normal mode by default (unlike the accel).
  // Datasheet §6.3: GYRO_LPM1 (0x11) power-on default = 0x00 = normal mode.

  // Read gyro chip ID (register 0x00, should be 0x0F for BMI085/088)
  Wire.beginTransmission(0x69);
  Wire.write(0x00);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)0x69, (uint8_t)1);
  uint8_t gyroChipId = Wire.read();
  Serial.print("Gyro chip ID: 0x");
  Serial.println(gyroChipId, HEX);

  // Set gyro range to ±2000°/s (register 0x0F, value 0x00) — datasheet §6.3.3
  //   0x00 = ±2000°/s  → 16.384 LSB/°/s
  //   0x01 = ±1000°/s  → 32.768 LSB/°/s
  //   0x02 = ±500°/s   → 65.536 LSB/°/s
  //   0x03 = ±250°/s   → 131.072 LSB/°/s
  //   0x04 = ±125°/s   → 262.144 LSB/°/s
  Wire.beginTransmission(0x69);
  Wire.write(0x0F);
  Wire.write(0x00); // ±2000°/s
  Wire.endTransmission();
  delay(2);

  // Read back to confirm
  Wire.beginTransmission(0x69);
  Wire.write(0x0F);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)0x69, (uint8_t)1);
  uint8_t gyroRange = Wire.read();
  Serial.print("Gyro range reg (0x0F): 0x");
  Serial.println(gyroRange, HEX);

  // // ── BNO08x ──
  // if (!bno08x.begin_I2C()) {
  //   Serial.println("Failed to find BNO08x chip");
  //   while (1) delay(10);
  // }
  // setReports();

  // ── ESKF ──
  rio::Params p = makeParams();
  eskf.setParams(p);

  rio::NominalState x0;
  x0.p_IR = p.p_IR;
  x0.q_IR = p.q_IR;
  eskf.reset(x0, P0_diag, 0.0f);

  Serial.println("RIO ESKF ready — waiting for attitude init from gravity");
}

// ──────────────────────────────────────────────────────────────
// Latest accel/gyro readings (buffered because the BNO08x
// delivers them as separate sensor events)
// ──────────────────────────────────────────────────────────────
static rio::Vec3 cur_acc = rio::Vec3::Zero();
static rio::Vec3 cur_gyr = rio::Vec3::Zero();
static bool      have_acc = false;
static bool      have_gyr = false;

// ──────────────────────────────────────────────────────────────
// Process one complete IMU sample through the ESKF
// ──────────────────────────────────────────────────────────────
static void processImu(const rio::Vec3& f_b, const rio::Vec3& w_b) {
  const float t = static_cast<float>(millis()) * 1e-3f;

  // Seed time on first call
  if (!time_initialized) {
    last_t = t;
    time_initialized = true;
    return;
  }

  // Initialize attitude from gravity on the first good reading
  if (!att_initialized) {
    if (!eskf.initAttitudeFromGravity(f_b, P0_diag, t)) {
      return;  // |acc| not close enough to 9.81 — wait
    }
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

  // ── Prediction ──
  eskf.predict(s, dt);
  eskf.insPropagation(s, dt);

  // ────────────────────────────────────────────────────────────
  // TODO: Radar Doppler correction
  // ────────────────────────────────────────────────────────────
  // When the radar is connected, buffer RadarDoppler measurements
  // and call:
  //
  //   rio::CorrectionResult res =
  //       eskf.correct(radar_buf.data(), radar_buf.size(), last_imu);
  //   radar_buf.clear();
  //
  // For now we just advance prior → posterior with no correction.
  // ────────────────────────────────────────────────────────────
  eskf.advancePriorToPosterior();

  publishState(t);
}

// ──────────────────────────────────────────────────────────────
// Arduino loop
// ──────────────────────────────────────────────────────────────
void loop() {
  // ── Read BMI085 accelerometer (0x19), 6 bytes from register 0x12 ──
  Wire.beginTransmission(0x19);
  Wire.write(0x12); // ACC_X_LSB register
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)0x19, (uint8_t)6);

  uint8_t buf[6];
  for (int i = 0; i < 6; i++) buf[i] = Wire.read();

  int16_t ax = (int16_t)(buf[0] | (buf[1] << 8));
  int16_t ay = (int16_t)(buf[2] | (buf[3] << 8));
  int16_t az = (int16_t)(buf[4] | (buf[5] << 8));

  // BMI085 ±4g range: full-scale = ±4g mapped to ±32768
  //   scale = 4.0 * 9.80665 / 32768 ≈ 0.001197 m/s² per LSB
  //
  // BMI085 ranges (register 0x41):
  //   0x00 = ±2g  → scale = 2.0 * 9.80665 / 32768
  //   0x01 = ±4g  → scale = 4.0 * 9.80665 / 32768
  //   0x02 = ±8g  → scale = 8.0 * 9.80665 / 32768
  //   0x03 = ±16g → scale = 16.0 * 9.80665 / 32768
  const float acc_scale = 8.0f * 9.80665f / 32768.0f;
  float acc_x = ax * acc_scale;
  float acc_y = ay * acc_scale;
  float acc_z = az * acc_scale;

  // Serial.print("Acc (m/s²): ");
  // Serial.print(acc_x, 4); Serial.print('\t');
  // Serial.print(acc_y, 4); Serial.print('\t');
  // Serial.print(acc_z, 4);
  // Serial.print("\traw Z: ");
  // Serial.println(az);

  delay(10);

  // ── Read BMI085 gyroscope (0x69), 6 bytes from register 0x02 ──
  Wire.beginTransmission(0x69);
  Wire.write(0x02); // GYRO_X_LSB register
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)0x69, (uint8_t)6);

  uint8_t gbuf[6];
  for (int i = 0; i < 6; i++) gbuf[i] = Wire.read();

  int16_t gx = (int16_t)(gbuf[0] | (gbuf[1] << 8));
  int16_t gy = (int16_t)(gbuf[2] | (gbuf[3] << 8));
  int16_t gz = (int16_t)(gbuf[4] | (gbuf[5] << 8));

  // BMI085 gyro ±2000°/s: sensitivity = 16.384 LSB/°/s
  //   scale = (1 / 16.384) * (π / 180) to get rad/s
  //
  // Gyro ranges (register 0x0F):
  //   0x00 = ±2000°/s → 16.384 LSB/°/s
  //   0x01 = ±1000°/s → 32.768 LSB/°/s
  //   0x02 = ±500°/s  → 65.536 LSB/°/s
  //   0x03 = ±250°/s  → 131.072 LSB/°/s
  //   0x04 = ±125°/s  → 262.144 LSB/°/s
  const float gyr_scale = (1.0f / 16.384f) * (M_PI / 180.0f); // → rad/s
  float gyr_x = gx * gyr_scale;
  float gyr_y = gy * gyr_scale;
  float gyr_z = gz * gyr_scale;

  Serial.print("Acc (m/s²): ");
  Serial.print(acc_x, 4); Serial.print('\t');
  Serial.print(acc_y, 4); Serial.print('\t');
  Serial.print(acc_z, 4);
  Serial.print("\tGyr (rad/s): ");
  Serial.print(gyr_x, 4); Serial.print('\t');
  Serial.print(gyr_y, 4); Serial.print('\t');
  Serial.println(gyr_z, 4);
}
