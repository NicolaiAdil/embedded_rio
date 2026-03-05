#include <Arduino.h>
#include <Wire.h>

// Arduino's binary.h defines B0, B1, B2, B3 which clash with Eigen
#undef B0
#undef B1
#undef B2
#undef B3

#include <rio/rio_eskf.h>
#include "bmi085.h"
#include "xwr6843aop.h"

// ──────────────────────────────────────────────────────────────
// ESKF filter
// ──────────────────────────────────────────────────────────────
static rio::RioEskf eskf;

static rio::Params makeParams() {
  rio::Params p;

  p.g_W = rio::Vec3(0.0f, 0.0f, -9.80665f);

  p.sigma_acc = 0.00000132435f;
  p.sigma_gyr = 0.0002443461f;
  p.sigma_ba  = 0.000001962f;
  p.sigma_bg  = 0.00000969627f;

  p.tau_ba = 700.0f;
  p.tau_bg =  450.0f;

  p.min_dt = 1e-4f;
  p.max_dt = 0.1f;

  p.q_IR = rio::Quat::Identity();
  p.p_IR = rio::Vec3::Zero();
  p.sigma_vr      = 0.06f;
  p.gating_enable = true;
  p.gate_nsigma   = 5.0f;
  p.vr_sign       = 1.0f;

  return p;
}

// Initial uncertianties for the error states
// TODO: change these, not very accurate.
static constexpr float P0_diag[21] = {
  1e-6f, 1e-6f, 1e-6f, // ego position (m)
  1e-2f, 1e-2f, 1e-2f, // ego velocity (m/s)
  1e-4f, 1e-4f, 1e-4f, // accelerometer bias (m/s²)
  1.1e-2f, 1.1e-2f, 1e-12f, // ego attitude (rad)
  1e-4f, 1e-4f, 1e-4f, // gyroscope bias (rad/s)
  2.0e-3f, 2.0e-3f, 2.0e-3f, // radar position relative to IMU (m)
  0.5f, 0.5f, 0.5f // radar attitude relative to IMU (rad)
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

  // eskf.advancePriorToPosterior();
}

// ──────────────────────────────────────────────────────────────
// Setup / loop
// ──────────────────────────────────────────────────────────────
static RadarFrame radarFrame;
int ledState = 0;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);   // LED pin on Teensy 4.1

  Serial.begin(115200);
  while (!Serial) delay(10);

  // --- IMU ---
  Wire.begin();
  Wire.setClock(400000);
  bmi085ScanI2C();
  if (!bmi085Init()) {
    Serial.println("BMI085 init failed");
    while (1) delay(100);
  }

  // --- ESKF ---
  rio::Params p = makeParams();
  eskf.setParams(p);
  rio::NominalState x0;
  x0.p_IR = p.p_IR;
  x0.q_IR = p.q_IR;
  eskf.reset(x0, P0_diag, 0.0f);

  // --- Radar ---
  if (!xwr6843aopInit()) {
    Serial.println("Radar config failed");
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
  if (!bmi085Read(acc, gyr)) {
    bmi085Recover();
    
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
  processImu(acc, gyr);

  // --- Radar ---
  xwr6843aopDrainCli();
  xwr6843aopUpdate(radarFrame);
  if (radarFrame.valid && radarFrame.numMeas > 0 && att_initialized) {
    rio::CorrectionResult res = eskf.correct(radarFrame.meas, radarFrame.numMeas, last_imu);

    // Prints
    if (res.n_rejected > 0) {
      Serial.print("ESKF correct: ");
      Serial.print(res.n_accepted); Serial.print(" accepted, ");
      Serial.print(res.n_rejected); Serial.print(" rejected, ");
      Serial.print(res.n_skipped);  Serial.println(" skipped");
    }
    const auto& x = eskf.getState();
    Serial.print("Nominal state: ");
    Serial.print("p_WI=[");
    Serial.print(x.p_WI.x(), 3); Serial.print(", ");
    Serial.print(x.p_WI.y(), 3); Serial.print(", ");
    Serial.print(x.p_WI.z(), 3); Serial.print("] m, ");
    Serial.print("v_WI=[");
    Serial.print(x.v_WI.x(), 3); Serial.print(", ");
    Serial.print(x.v_WI.y(), 3); Serial.print(", ");
    Serial.print(x.v_WI.z(), 3); Serial.println("] m/s");
  }
  // Send state update...
}
