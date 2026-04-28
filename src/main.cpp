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

// ──────────────────────────────────────────────────────────────
// IMU type selection — change this line to swap hardware
// ──────────────────────────────────────────────────────────────
// static constexpr ImuType IMU_TYPE = ImuType::BMI085;
static constexpr ImuType IMU_TYPE = ImuType::BMI088;

// ──────────────────────────────────────────────────────────────
// ESKF filter
// ──────────────────────────────────────────────────────────────
static rio::RioEskf   eskf;
static rio::Params    g_params;


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
  // p.p_IR = rio::Vec3(-0.065f, 0.043f, 0.020f);
  p.p_IR = rio::Vec3(-0.065f, 0.025013f, 0.020f);
  // p.p_IR = rio::Vec3(0.0f, 0.0f, 0.0f);
  p.sigma_vr      = 0.058f; // 0.038f
  p.gating_enable = true;
  p.gate_nsigma   = 5.0f;
  p.vr_sign       = 1.0f;
  return p;
}

// Initial uncertainties for the error states
static constexpr float P0_diag[21] = {
  1e-6f  , 1e-6f  , 1e-6f  , // ego position (m)
  1e-1f  , 1e-1f  , 1e-1f  , // ego velocity (m/s)
  1e-4f  , 1e-4f  , 1e-4f  , // accelerometer bias (m/s²)
  1.1e-3f, 1.1e-3f, 1e-8f , // ego attitude (rad): roll/pitch from gravity, yaw unknown
  1e-5f  , 1e-5f  , 1e-5f  , // gyroscope bias (rad/s)
  2.0e-5f, 2.0e-5f, 2.0e-5f, // radar position relative to IMU (m)
  1.0e-2f, 1.0e-2f, 1.0e-2f   , // radar attitude relative to IMU (rad)
};

// ──────────────────────────────────────────────────────────────
// MAVLink odometry output (Serial2 → Telem1)
// ──────────────────────────────────────────────────────────────
static uint8_t mav_buf[MAVLINK_MAX_PACKET_LEN];

// Fill the 21-element upper-triangle of a 6x6 covariance from selected rows/cols of P.
// r[6] maps output row/col index → P matrix row/col index. NaN columns set that row/col to NaN.
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

static void sendOdometry(const rio::NominalState& x, const rio::Mat21& P, float t_sec, int8_t quality) {
  // 180° pitch = quaternion (w=0, x=0, y=1, z=0)
  static const rio::Quat q_t(0.0f, 0.0f, 1.0f, 0.0f);  // Eigen (w,x,y,z)

  const rio::Vec3 p_out = q_t * x.p_WI;
  const rio::Vec3 v_body = q_t * (x.q_WI.inverse() * x.v_WI);
  const rio::Quat q_out = q_t * x.q_WI * q_t.inverse();

  mavlink_odometry_t odom{};
  odom.time_usec      = (uint64_t)(t_sec * 1e6f);
  odom.frame_id       = MAV_FRAME_LOCAL_NED;
  odom.child_frame_id = MAV_FRAME_BODY_FRD;

  odom.x = p_out.x();
  odom.y = p_out.y();
  odom.z = p_out.z();

  // Quaternion [w, x, y, z]
  odom.q[0] = q_out.w();
  odom.q[1] = q_out.x();
  odom.q[2] = q_out.y();
  odom.q[3] = q_out.z();

  odom.vx = v_body.x();
  odom.vy = v_body.y();
  odom.vz = v_body.z();

  odom.quality = quality;

  // Angular velocity not available — mark unknown
  odom.rollspeed  = NAN;
  odom.pitchspeed = NAN;
  odom.yawspeed   = NAN;

  // pose_covariance: upper triangle of [pos(0:3), att(9:12)] from P
  {
    const int    r[6]       = {0, 1, 2, 9, 10, 11};
    bool         nan_col[6] = {false, false, false, false, false, false};
    fillCov6(odom.pose_covariance, P, r, nan_col);
  }

  // velocity_covariance: upper triangle of [vel(3:6), ang_vel=NaN]
  {
    const int    r[6]       = {3, 4, 5, 0, 0, 0};   // last 3 unused (NaN)
    bool         nan_col[6] = {false, false, false, true, true, true};
    fillCov6(odom.velocity_covariance, P, r, nan_col);
  }

  odom.reset_counter  = 0;
  odom.estimator_type = MAV_ESTIMATOR_TYPE_VISION;

  mavlink_message_t msg;
  mavlink_msg_odometry_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &odom);
  uint16_t len = mavlink_msg_to_send_buffer(mav_buf, &msg);
  Serial2.write(mav_buf, len);
}

static bool  att_initialized  = false;
static bool  time_initialized = false;
static float last_t           = 0.0f;
static rio::ImuSample last_imu{};

// ──────────────────────────────────────────────────────────────
// IMU rate limiter
// ──────────────────────────────────────────────────────────────
static constexpr uint32_t IMU_HZ          = 200;
static constexpr uint32_t IMU_PERIOD_US   = 1000000UL / IMU_HZ;
static uint32_t s_imu_last_us             = 0;

// ──────────────────────────────────────────────────────────────
// Rate tracking
// ──────────────────────────────────────────────────────────────
static uint32_t s_imu_count   = 0;
static uint32_t s_radar_count = 0;
static uint32_t s_rate_t_ms   = 0;

// ──────────────────────────────────────────────────────────────
// IMU processing
// ──────────────────────────────────────────────────────────────
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

  // Serial.print("gyro: [");
  // Serial.print(w_b.x(), 4); Serial.print(", ");
  // Serial.print(w_b.y(), 4); Serial.print(", ");
  // Serial.print(w_b.z(), 4); Serial.println("]");

  if (dt >= g_params.min_dt && dt <= g_params.max_dt) {
    eskf.predict(s, dt);
    eskf.insPropagation(s, dt);
  }

  s_imu_count++;
}

// ──────────────────────────────────────────────────────────────
// Setup / loop
// ──────────────────────────────────────────────────────────────
static RadarFrame radarFrame;
static int ledState = 0;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

#if USB_PRINT_ENABLED
  Serial.begin(115200);
  while (!Serial) delay(10);
#endif
  Serial2.begin(921600);  // TX2/RX2 → Telem1

  // --- IMU ---
  Wire.begin();
  Wire.setClock(400000);
  bmi08xScanI2C();
  if (!bmi08xInit(IMU_TYPE)) {
#if USB_PRINT_ENABLED
    Serial.println("BMI08x init failed");
#endif
    while (1) delay(100);
  }
  delay(200);

  // --- ESKF ---
  g_params = makeParams();
  eskf.setParams(g_params);
  rio::NominalState x0;
  x0.p_IR = g_params.p_IR;
  x0.q_IR = g_params.q_IR;
  eskf.reset(x0, P0_diag, 0.0f);

  // --- Radar ---
//   if (!xwr6843aopInit()) {
// #if USB_PRINT_ENABLED
//     Serial.println("Radar config failed");
// #endif
//     while (1) delay(100); //TODO: try to recover?
//   } else {
// #if USB_PRINT_ENABLED
//     Serial.println("Radar configured OK!");
// #endif
//   }

  // --- Barometer ---
  if (!bmp581Init()) {
#if USB_PRINT_ENABLED
    Serial.println("BMP581 init failed");
#endif
  }

#if USB_PRINT_ENABLED
  Serial.println("RIO ESKF ready");
#endif
  digitalWrite(LED_BUILTIN, HIGH);
  ledState = 1;

  s_rate_t_ms = millis();
}

void loop() {
  // --- IMU ---
  rio::Vec3 acc, gyr;
  if (!bmi08xRead(acc, gyr)) {
    bmi08xRecover();
    if (ledState == 1) {
      digitalWrite(LED_BUILTIN, LOW);
      ledState = 0;
    }
    return;
  }

  if (ledState == 0) {
    digitalWrite(LED_BUILTIN, HIGH);
    ledState = 1;
  }

  if (!acc.allFinite() || !gyr.allFinite()) return;

  const uint32_t now_us = micros();
  if (now_us - s_imu_last_us >= IMU_PERIOD_US) {
    s_imu_last_us = now_us;
    processImu(acc, gyr);
  }

  // --- Radar ---
  xwr6843aopDrainCli();
  xwr6843aopUpdate(radarFrame);
  if (radarFrame.valid && radarFrame.numRaw > 0 && att_initialized) { 
    // xwr6843aopPrintRaw(radarFrame);

    static rio::RadarDoppler doppler[RadarFrame::MAX_POINTS];
    const float t_r = static_cast<float>(millis()) * 1e-3f;
    for (uint32_t i = 0; i < radarFrame.numRaw; i++) {
      doppler[i].t     = t_r;
      doppler[i].u_R   = rio::Vec3(radarFrame.raw[i].x,
                                   radarFrame.raw[i].y,
                                   radarFrame.raw[i].z);
      doppler[i].vr    = radarFrame.raw[i].vr;
      doppler[i].sigma = g_params.sigma_vr;
    }

    rio::CorrectionResult res = eskf.correct(doppler, radarFrame.numRaw, last_imu);
    s_radar_count++;

    const uint32_t total = res.n_accepted + res.n_rejected + res.n_skipped;
    const int8_t quality = (total > 0) ? static_cast<int8_t>(res.n_accepted * 100u / total) : 0;

    if (res.n_rejected > 0) {
      Serial.print("ESKF correct: ");
      Serial.print(res.n_accepted); Serial.print(" accepted, ");
      Serial.print(res.n_rejected); Serial.print(" rejected, ");
      Serial.print(res.n_skipped);  Serial.println(" skipped");
    }

    const auto& x = eskf.getState();
    const auto& P = eskf.getCovariance();
#if !USB_PRINT_ENABLED
    sendOdometry(x, P, t_r, quality);
#else
    // Position
    Serial.print("p_WI=[");
    Serial.print(x.p_WI.x(), 4); Serial.print(", ");
    Serial.print(x.p_WI.y(), 4); Serial.print(", ");
    Serial.print(x.p_WI.z(), 4); Serial.println("]");

    // Velocity
    Serial.print("v_WI=[");
    Serial.print(x.v_WI.x(), 4); Serial.print(", ");
    Serial.print(x.v_WI.y(), 4); Serial.print(", ");
    Serial.print(x.v_WI.z(), 4); Serial.println("]");

    // Attitude quaternion
    Serial.print("q_WI=[");
    Serial.print(x.q_WI.w(), 6); Serial.print(", ");
    Serial.print(x.q_WI.x(), 6); Serial.print(", ");
    Serial.print(x.q_WI.y(), 6); Serial.print(", ");
    Serial.print(x.q_WI.z(), 6); Serial.println("]");

    // Accelerometer bias
    Serial.print("b_a=[");
    Serial.print(x.b_a.x(), 6); Serial.print(", ");
    Serial.print(x.b_a.y(), 6); Serial.print(", ");
    Serial.print(x.b_a.z(), 6); Serial.println("]");

    // Gyroscope bias
    Serial.print("b_g=[");
    Serial.print(x.b_g.x(), 6); Serial.print(", ");
    Serial.print(x.b_g.y(), 6); Serial.print(", ");
    Serial.print(x.b_g.z(), 6); Serial.println("]");

    // Radar-IMU translation extrinsic
    Serial.print("p_IR=[");
    Serial.print(x.p_IR.x(), 6); Serial.print(", ");
    Serial.print(x.p_IR.y(), 6); Serial.print(", ");
    Serial.print(x.p_IR.z(), 6); Serial.println("]");

    // Radar-IMU rotation extrinsic
    Serial.print("q_IR=[");
    Serial.print(x.q_IR.w(), 6); Serial.print(", ");
    Serial.print(x.q_IR.x(), 6); Serial.print(", ");
    Serial.print(x.q_IR.y(), 6); Serial.print(", ");
    Serial.print(x.q_IR.z(), 6); Serial.println("]");

    // Covariance std devs — per group: avg(σ_x,σ_y) | σ_z
    // const auto& P = eskf.getCovariance();
    auto s = [&](int i){ return sqrtf(fabsf(P(i,i))); };
    auto xy = [&](int i){ return 0.5f*(s(i)+s(i+1)); };
    // p    v    b_a  att  b_g  p_IR q_IR
    Serial.print("cov_xy=[");
    Serial.print(xy( 0),4); Serial.print(", "); Serial.print(xy( 3),4); Serial.print(", ");
    Serial.print(xy( 6),4); Serial.print(", "); Serial.print(xy( 9),4); Serial.print(", ");
    Serial.print(xy(12),4); Serial.print(", "); Serial.print(xy(15),4); Serial.print(", ");
    Serial.print(xy(18),4); Serial.println("]");
    Serial.print("cov_z= [");
    Serial.print(s( 2),4); Serial.print(", "); Serial.print(s( 5),4); Serial.print(", ");
    Serial.print(s( 8),4); Serial.print(", "); Serial.print(s(11),4); Serial.print(", ");
    Serial.print(s(14),4); Serial.print(", "); Serial.print(s(17),4); Serial.print(", ");
    Serial.print(s(20),4); Serial.println("]");
  #endif
  }

  // --- Rate logging + barometer print (once per second) ---
  const uint32_t now_ms = millis();
  if (now_ms - s_rate_t_ms >= 1000) {
    const float dt_s = (now_ms - s_rate_t_ms) * 1e-3f;
#if USB_PRINT_ENABLED
    Serial.print("RATES imu_hz=");
    Serial.print(s_imu_count   / dt_s, 1);
    Serial.print(" radar_hz=");
    Serial.println(s_radar_count / dt_s, 1);

    float baro_temp, baro_press;
    if (bmp581Read(baro_temp, baro_press)) {
      Serial.print("BARO temp=");
      Serial.print(baro_temp, 2);
      Serial.print(" C  press=");
      Serial.print(baro_press, 2);
      Serial.print(" Pa (");
      Serial.print(baro_press / 100.0f, 2);
      Serial.println(" hPa)");
    } else {
      Serial.println("BARO no data ready");
    }
#endif
    s_imu_count   = 0;
    s_radar_count = 0;
    s_rate_t_ms   = now_ms;
  }
}