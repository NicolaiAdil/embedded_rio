#include <Arduino.h>
#include <Wire.h>
#include <common/mavlink.h>

#undef B0
#undef B1
#undef B2
#undef B3

#include <rio/rio_eskf.h>
#include "bmi08x.h"
#include "xwr6843aop.h"
#include "bmp581.h"

#include "config.h"
#include "sd_logger.h"

// static constexpr ImuType IMU_TYPE = ImuType::BMI085;
static constexpr ImuType IMU_TYPE = ImuType::BMI088;

static rio::RioEskf eskf;
static rio::Params  g_params;

static rio::Params makeParams() {
  rio::Params p;

  p.g_W = rio::Vec3(0.0f, 0.0f, -9.80665f);

  p.sigma_acc = 2.2563e-3f;
  p.sigma_ba  = 2.2563e-4f;

  p.sigma_gyr = 2.443461e-4f;
  p.sigma_bg  = 2.443461e-5f;

  p.tau_ba = 700.0f;
  p.tau_bg = 450.0f;

  p.min_dt = 1e-6f;
  p.max_dt = 0.1f;

  p.q_IR = rio::Quat(0.68301f, -0.18301f, -0.18301f, 0.68301f);  // Eigen (w,x,y,z)
  p.p_IR = rio::Vec3(-0.065f, 0.025013f, 0.020f);

  return p;
}

// Per-modality measurement parameters and persistent measurement instances.
static const rio::RadarDopplerMeasurement::Params g_radar_p{
    /*sigma_vr=*/0.038f, /*vr_sign=*/1.0f,
    /*gate_nsigma=*/5.0f, /*gating=*/true};

static rio::BarometerDiffMeasurement g_baro_meas(
    rio::BarometerDiffMeasurement::Params{
        /*sigma_dz=*/0.3f, /*z_sign=*/1.0f,
        /*gate_nsigma=*/5.0f, /*gating=*/true});

static constexpr float P0_diag[21] = {
  1e-6f  , 1e-6f  , 1e-6f  , // ego position (m)
  1e-1f  , 1e-1f  , 1e-1f  , // ego velocity (m/s)
  1e-2f  , 1e-2f  , 1e-2f  , // accelerometer bias (m/s²)
  1.1e-3f, 1.1e-3f, 1e-8f , // ego attitude (rad): roll/pitch from gravity, yaw unknown
  1e-4f  , 1e-4f  , 1e-4f  , // gyroscope bias (rad/s)
  2.0e-5f, 2.0e-5f, 2.0e-5f, // radar position relative to IMU (m)
  1.0e-2f, 1.0e-2f, 1.0e-2f, // radar attitude relative to IMU (rad)
};

static bool           att_initialized  = false;
static bool           time_initialized = false;
static float          last_t           = 0.0f;
static rio::ImuSample last_imu{};

static uint32_t s_imu_count   = 0;
static uint32_t s_radar_count = 0;
static uint32_t s_baro_count  = 0;
static uint32_t s_rate_t_ms   = 0;

// Cached so the 1-Hz logger doesn't trigger another I2C read.
static float s_baro_temp_c   = NAN;
static float s_baro_press_pa = NAN;
static bool  s_baro_valid    = false;

static uint8_t mav_buf[MAVLINK_MAX_PACKET_LEN];

// Fill the 21-element upper triangle of a 6x6 covariance from selected rows/cols of P.
// r[6] maps output index → P index. nan_col[i]=true blanks row i and col i.
static void fillCov6(float out[21], const rio::Mat21& P,
                     const int r[6], bool nan_col[6]) {
  int k = 0;
  for (int i = 0; i < 6; i++) {
    for (int j = i; j < 6; j++, k++) {
      if (nan_col[i] || nan_col[j]) {
        out[k] = NAN;
      } else {
        out[k] = P(r[i], r[j]);
      }
    }
  }
}

static void sendOdometry(const rio::NominalState& x, const rio::Mat21& P,
                         float t_sec, int8_t quality) {
  // 180° pitch rotation: Eigen (w,x,y,z) = (0, 0, 1, 0).
  static const rio::Quat q_t(0.0f, 0.0f, 1.0f, 0.0f);

  const rio::Vec3 p_out  = q_t * x.p_WI;
  const rio::Vec3 v_body = q_t * (x.q_WI.inverse() * x.v_WI);
  const rio::Quat q_out  = q_t * x.q_WI * q_t.inverse();

  mavlink_odometry_t odom{};
  odom.time_usec      = (uint64_t)(t_sec * 1e6f);
  odom.frame_id       = MAV_FRAME_LOCAL_NED;
  odom.child_frame_id = MAV_FRAME_BODY_FRD;

  odom.x = p_out.x();
  odom.y = p_out.y();
  odom.z = p_out.z();

  odom.q[0] = q_out.w();
  odom.q[1] = q_out.x();
  odom.q[2] = q_out.y();
  odom.q[3] = q_out.z();

  odom.vx = v_body.x();
  odom.vy = v_body.y();
  odom.vz = v_body.z();

  odom.quality = quality;

  // Angular velocity not available.
  odom.rollspeed  = NAN;
  odom.pitchspeed = NAN;
  odom.yawspeed   = NAN;

  // pose_covariance: upper triangle of [pos(0:3), att(9:12)] from P.
  {
    const int r[6]       = {0, 1, 2, 9, 10, 11};
    bool      nan_col[6] = {false, false, false, false, false, false};
    fillCov6(odom.pose_covariance, P, r, nan_col);
  }

  // velocity_covariance: upper triangle of [vel(3:6), ang_vel=NaN].
  {
    const int r[6]       = {3, 4, 5, 0, 0, 0};
    bool      nan_col[6] = {false, false, false, true, true, true};
    fillCov6(odom.velocity_covariance, P, r, nan_col);
  }

  odom.reset_counter  = 0;
  odom.estimator_type = MAV_ESTIMATOR_TYPE_VISION;

  mavlink_message_t msg;
  mavlink_msg_odometry_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &odom);
  uint16_t len = mavlink_msg_to_send_buffer(mav_buf, &msg);
  Serial2.write(mav_buf, len);
}

#if USB_PRINT_ENABLED
static void printState(const rio::NominalState& x, const rio::Mat21& P) {
  Serial.print("p_WI=[");
  Serial.print(x.p_WI.x(), 6); Serial.print(", ");
  Serial.print(x.p_WI.y(), 6); Serial.print(", ");
  Serial.print(x.p_WI.z(), 6); Serial.println("]");

  Serial.print("v_WI=[");
  Serial.print(x.v_WI.x(), 6); Serial.print(", ");
  Serial.print(x.v_WI.y(), 6); Serial.print(", ");
  Serial.print(x.v_WI.z(), 6); Serial.println("]");

  Serial.print("q_WI=[");
  Serial.print(x.q_WI.w(), 6); Serial.print(", ");
  Serial.print(x.q_WI.x(), 6); Serial.print(", ");
  Serial.print(x.q_WI.y(), 6); Serial.print(", ");
  Serial.print(x.q_WI.z(), 6); Serial.println("]");

  Serial.print("b_a=[");
  Serial.print(x.b_a.x(), 6); Serial.print(", ");
  Serial.print(x.b_a.y(), 6); Serial.print(", ");
  Serial.print(x.b_a.z(), 6); Serial.println("]");

  Serial.print("b_g=[");
  Serial.print(x.b_g.x(), 6); Serial.print(", ");
  Serial.print(x.b_g.y(), 6); Serial.print(", ");
  Serial.print(x.b_g.z(), 6); Serial.println("]");

  Serial.print("p_IR=[");
  Serial.print(x.p_IR.x(), 6); Serial.print(", ");
  Serial.print(x.p_IR.y(), 6); Serial.print(", ");
  Serial.print(x.p_IR.z(), 6); Serial.println("]");

  Serial.print("q_IR=[");
  Serial.print(x.q_IR.w(), 6); Serial.print(", ");
  Serial.print(x.q_IR.x(), 6); Serial.print(", ");
  Serial.print(x.q_IR.y(), 6); Serial.print(", ");
  Serial.print(x.q_IR.z(), 6); Serial.println("]");

  // Per-group std devs: cov_xy = avg(σ_x, σ_y), cov_z = σ_z.
  // Groups: p, v, b_a, att, b_g, p_IR, q_IR.
  auto s  = [&](int i){ return sqrtf(fabsf(P(i, i))); };
  auto xy = [&](int i){ return 0.5f * (s(i) + s(i + 1)); };
  Serial.print("cov_xy=[");
  Serial.print(xy( 0), 6); Serial.print(", "); Serial.print(xy( 3), 6); Serial.print(", ");
  Serial.print(xy( 6), 6); Serial.print(", "); Serial.print(xy( 9), 6); Serial.print(", ");
  Serial.print(xy(12), 6); Serial.print(", "); Serial.print(xy(15), 6); Serial.print(", ");
  Serial.print(xy(18), 6); Serial.println("]");
  Serial.print("cov_z= [");
  Serial.print(s( 2), 6); Serial.print(", "); Serial.print(s( 5), 6); Serial.print(", ");
  Serial.print(s( 8), 6); Serial.print(", "); Serial.print(s(11), 6); Serial.print(", ");
  Serial.print(s(14), 6); Serial.print(", "); Serial.print(s(17), 6); Serial.print(", ");
  Serial.print(s(20), 6); Serial.println("]");
}
#endif

static void publishState(float t, int8_t quality) {
  const auto& x = eskf.getState();
  const auto& P = eskf.getCovariance();
#if USB_PRINT_ENABLED
  printState(x, P);
#else
  sendOdometry(x, P, t, quality);
#endif
}

static constexpr uint32_t IMU_HZ        = 200;
static constexpr uint32_t IMU_PERIOD_US = 1000000UL / IMU_HZ;
static uint32_t s_imu_last_us           = 0;

static constexpr uint32_t BARO_HZ        = 50;
static constexpr uint32_t BARO_PERIOD_US = 1000000UL / BARO_HZ;
static uint32_t s_baro_last_us           = 0;

static void processImu(const rio::Vec3& f_b, const rio::Vec3& w_b) {
  const float t = static_cast<float>(millis()) * 1e-3f;

#if SD_LOG_ENABLED
  sdLoggerLogImu(t, f_b, w_b);
#endif

  if (!time_initialized) {
    last_t = t;
    time_initialized = true;
    return;
  }

  if (!att_initialized) {
    if (!eskf.initAttitudeFromGravity(f_b, P0_diag, t)) return;
    att_initialized = true;
    last_t = t;
#if USB_PRINT_ENABLED
    Serial.println("Attitude initialized from gravity");
#endif
    return;
  }

  const float dt = t - last_t;
  last_t = t;

  rio::ImuSample s;
  s.t   = t;
  s.acc = f_b;
  s.gyr = w_b;
  last_imu = s;

  if (dt >= g_params.min_dt && dt <= g_params.max_dt) {
    eskf.predict(s, dt);
    eskf.insPropagation(s, dt);
  }

  s_imu_count++;
}

static void processRadar(const RadarFrame& frame) {
  if (!frame.valid || frame.numRaw == 0 || !att_initialized) return;

  const float t_r = static_cast<float>(millis()) * 1e-3f;

#if SD_LOG_ENABLED
  sdLoggerLogRadar(t_r, frame);
#endif

  s_radar_count++;

  rio::MeasurementContext ctx{&last_imu};
  uint32_t n_acc = 0, n_rej = 0, n_skp = 0;
  for (uint32_t i = 0; i < frame.numRaw; i++) {
    rio::RadarDopplerMeasurement m(
        g_radar_p,
        rio::Vec3(frame.raw[i].x, frame.raw[i].y, frame.raw[i].z),
        frame.raw[i].vr);
    const rio::ScalarUpdate u = eskf.applyScalar(m, ctx);
    switch (u.status) {
      case rio::ScalarUpdate::Accepted: ++n_acc; break;
      case rio::ScalarUpdate::Rejected: ++n_rej; break;
      default:                          ++n_skp; break;
    }
  }
  // Match the old batch behavior: if no point was accepted, snap P_hat_
  // to the predicted prior so getCovariance() reflects the latest predict.
  if (n_acc == 0) eskf.advancePriorToPosterior();

  const uint32_t total   = n_acc + n_rej + n_skp;
  const int8_t   quality = (total > 0)
      ? static_cast<int8_t>(n_acc * 100u / total)
      : 0;

#if USB_PRINT_ENABLED
  if (n_rej > 0 || n_skp > 0) {
    Serial.print("ESKF correct: ");
    Serial.print(n_acc); Serial.print(" accepted, ");
    Serial.print(n_rej); Serial.print(" rejected, ");
    Serial.print(n_skp); Serial.println(" skipped");
  }
#endif

  publishState(t_r, quality);
}

static void processBaro() {
  const uint32_t now_us = micros();
  if (now_us - s_baro_last_us < BARO_PERIOD_US) return;
  s_baro_last_us = now_us;

  float temp_c, press_pa;
  if (!bmp581Read(temp_c, press_pa)) {
    // Recovery just ran inside bmp581Read; the differential anchor held
    // by g_baro_meas no longer matches whatever pressure the chip will
    // report next, so drop it to avoid a wild Δz on the first reading.
    g_baro_meas.resetAnchor();
    return;
  }

  s_baro_temp_c   = temp_c;
  s_baro_press_pa = press_pa;
  s_baro_valid    = true;

#if SD_LOG_ENABLED
  {
    const float t_s = static_cast<float>(millis()) * 1e-3f;
    sdLoggerLogBaro(t_s, temp_c, press_pa);
  }
#endif

  s_baro_count++;

  if (!att_initialized) return;

  rio::BarometerSample baro;
  baro.t           = static_cast<float>(millis()) * 1e-3f;
  baro.pressure_pa = press_pa;
  baro.temp_c      = temp_c;

  g_baro_meas.setSample(baro);
  const rio::ScalarUpdate u = eskf.applyScalar(g_baro_meas);

#if USB_PRINT_ENABLED
  if (u.status == rio::ScalarUpdate::Rejected) {
    Serial.print("BARO rejected: dz_meas=");
    Serial.print(g_baro_meas.lastDzMeas(), 3);
    Serial.print(" dz_pred=");
    Serial.print(g_baro_meas.lastDzPred(), 3);
    Serial.print(" residual=");
    Serial.println(u.residual, 3);
  }
#endif

  if (u.status == rio::ScalarUpdate::Accepted) {
    publishState(baro.t, /*quality=*/100);
  }
}

static void printRates(uint32_t now_ms) {
  if (now_ms - s_rate_t_ms < 1000) return;
  const float dt_s = (now_ms - s_rate_t_ms) * 1e-3f;

#if USB_PRINT_ENABLED
  Serial.print("RATES imu_hz=");
  Serial.print(s_imu_count   / dt_s, 1);
  Serial.print(" radar_hz=");
  Serial.print(s_radar_count / dt_s, 1);
  Serial.print(" baro_hz=");
  Serial.println(s_baro_count / dt_s, 1);

  if (s_baro_valid) {
    Serial.print("BARO temp=");
    Serial.print(s_baro_temp_c, 2);
    Serial.print(" C  press=");
    Serial.print(s_baro_press_pa, 2);
    Serial.print(" Pa (");
    Serial.print(s_baro_press_pa / 100.0f, 2);
    Serial.println(" hPa)");
  }
#endif

  s_imu_count   = 0;
  s_radar_count = 0;
  s_baro_count  = 0;
  s_rate_t_ms   = now_ms;
}

static int s_led = 0;
static void setLed(int v) {
  if (s_led == v) return;
  digitalWrite(LED_BUILTIN, v ? HIGH : LOW);
  s_led = v;
}

static RadarFrame radarFrame;

static void setupSensors() {
  Wire.begin();
  Wire.setClock(400000);
  bmi08xScanI2C();
  if (!bmi08xInit(IMU_TYPE)) {
#if USB_PRINT_ENABLED
    Serial.println("BMI08x init failed");
#endif
    while (1) delay(100);
  }

  if (!bmp581Init()) {
#if USB_PRINT_ENABLED
    Serial.println("BMP581 init failed");
#endif
    while (1) delay(100);
  }
  delay(200);

  if (!xwr6843aopInit()) {
    Serial.println("Radar config failed");
    while (1) delay(100);
  }

}

static void setupEskf() {
  g_params = makeParams();
  eskf.setParams(g_params);

  rio::NominalState x0;
  x0.p_IR = g_params.p_IR;
  x0.q_IR = g_params.q_IR;
  eskf.reset(x0, P0_diag, 0.0f);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

#if USB_PRINT_ENABLED
  Serial.begin(115200);
  while (!Serial) delay(10);

  // Teensy 4.x preserves fault info in RAM2 across a hard fault. If the
  // previous run crashed, dump the fault address / PC / stack so we know
  // exactly where it died.
  if (CrashReport) {
    Serial.println("──── PREVIOUS CRASH ────");
    Serial.print(CrashReport);
    Serial.println("────────────────────────");
  }
#endif
  Serial2.begin(921600);  // Telem1

  setupSensors();
  setupEskf();

#if SD_LOG_ENABLED
  sdLoggerInit();
#endif

#if USB_PRINT_ENABLED
  Serial.println("RIO ESKF ready");
#endif
  setLed(1);
  s_rate_t_ms = millis();
}

void loop() {
  rio::Vec3 acc, gyr;
  if (!bmi08xRead(acc, gyr)) {
    bmi08xRecover();
    setLed(0);
    return;
  }
  setLed(1);
  if (!acc.allFinite() || !gyr.allFinite()) return;

  const uint32_t now_us = micros();
  if (now_us - s_imu_last_us >= IMU_PERIOD_US) {
    s_imu_last_us = now_us;
    processImu(acc, gyr);
  }

  xwr6843aopDrainCli();
  xwr6843aopUpdate(radarFrame);
  processRadar(radarFrame);

  processBaro();

  printRates(millis());

#if SD_LOG_ENABLED
  {
    static uint32_t s_flush_last_ms = 0;
    const uint32_t  now_ms          = millis();
    if (now_ms - s_flush_last_ms >= SD_LOG_FLUSH_INTERVAL_S * 1000UL) {
      s_flush_last_ms = now_ms;
      sdLoggerFlush();
    }
  }
#endif
}
