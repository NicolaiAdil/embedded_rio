#include <Arduino.h>
#include <Wire.h>

#undef B0
#undef B1
#undef B2
#undef B3

#include <rio/rio_eskf.h>
#include "bmi08x.h"
#include "xwr6843aop.h"

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

  p.sigma_acc = 1.71675e-3f;
  p.sigma_gyr = 2.443461e-4f;
  p.sigma_ba  = 1.71675e-4f;
  p.sigma_bg  = 2.443461e-5f;

  p.tau_ba = 700.0f;
  p.tau_bg = 450.0f;

  p.min_dt = 1e-4f;
  p.max_dt = 0.1f;

  // Radar frame: x=forward, y=right, z=down (FRU).
  // IMU frame:   x=forward, y=right, z=up  (FRU).
  p.q_IR = rio::Quat(0.68301f, -0.18301f, -0.18301f, 0.68301f);  // Eigen (w,x,y,z)
  p.p_IR = rio::Vec3(-0.065f, 0.043f, 0.020f);
  p.sigma_vr      = 0.038f; // 0.038f
  p.gating_enable = true;
  p.gate_nsigma   = 5.0f;
  p.vr_sign       = 1.0f;

  return p;
}

// Initial uncertainties for the error states
static constexpr float P0_diag[21] = {
  1e-6f  , 1e-6f  , 1e-6f  , // ego position (m)
  1e-2f  , 1e-2f  , 1e-2f  , // ego velocity (m/s)
  1e-4f  , 1e-4f  , 1e-4f  , // accelerometer bias (m/s²)
  1.1e-2f, 1.1e-2f, 1e-12f , // ego attitude (rad)
  1e-4f  , 1e-4f  , 1e-4f  , // gyroscope bias (rad/s)
  2.0e-3f, 2.0e-3f, 2.0e-3f, // radar position relative to IMU (m)
  1.0e-2f, 1.0e-2f, 1.0e-2f   , // radar attitude relative to IMU (rad)
};

static bool  att_initialized  = false;
static bool  time_initialized = false;
static float last_t           = 0.0f;
static rio::ImuSample last_imu{};

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
}

// ──────────────────────────────────────────────────────────────
// Setup / loop
// ──────────────────────────────────────────────────────────────
static RadarFrame radarFrame;
static int ledState = 0;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);
  while (!Serial) delay(10);

  // --- IMU ---
  Wire.begin();
  Wire.setClock(400000);
  bmi08xScanI2C();
  if (!bmi08xInit(IMU_TYPE)) {
    Serial.println("BMI08x init failed");
    while (1) delay(100);
  }

  // --- ESKF ---
  g_params = makeParams();
  eskf.setParams(g_params);
  rio::NominalState x0;
  x0.p_IR = g_params.p_IR;
  x0.q_IR = g_params.q_IR;
  eskf.reset(x0, P0_diag, 0.0f);

  // --- Radar ---
  if (!xwr6843aopInit()) {
    Serial.println("Radar config failed");
    while (1) delay(100); //TODO: try to recover?
  } else {
    Serial.println("Radar configured OK!");
  }

  Serial.println("RIO ESKF ready");
  digitalWrite(LED_BUILTIN, HIGH);
  ledState = 1;
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
  // Serial.print("acc: [");
  // Serial.print(acc.x(), 3); Serial.print(", ");
  // Serial.print(acc.y(), 3); Serial.print(", ");
  // Serial.print(acc.z(), 3); Serial.print("] m/s², gyr: [");
  // Serial.print(gyr.x(), 3); Serial.print(", ");
  // Serial.print(gyr.y(), 3); Serial.print(", ");
  // Serial.print(gyr.z(), 3); Serial.println("] rad/s");  

  if (ledState == 0) {
    digitalWrite(LED_BUILTIN, HIGH);
    ledState = 1;
  }
  processImu(acc, gyr);

  // --- Radar ---
  xwr6843aopDrainCli();
  xwr6843aopUpdate(radarFrame);
  if (radarFrame.valid && radarFrame.numRaw > 0 && att_initialized) {
    xwr6843aopPrintRaw(radarFrame);

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

    if (res.n_rejected > 0) {
      Serial.print("ESKF correct: ");
      Serial.print(res.n_accepted); Serial.print(" accepted, ");
      Serial.print(res.n_rejected); Serial.print(" rejected, ");
      Serial.print(res.n_skipped);  Serial.println(" skipped");
    }

    const auto& x = eskf.getState();

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
  }
}