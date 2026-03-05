#pragma once
// ──────────────────────────────────────────────────────────────
// BMI085 IMU driver for Teensy 4.1
//
// Wiring (I2C, SDO1/SDO2 → VDDIO):
//   Accel address: 0x19
//   Gyro  address: 0x69
//   SDA: pin 18
//   SCL: pin 19
//   I2C clock: 400 kHz
// ──────────────────────────────────────────────────────────────

#include <Arduino.h>

// We only need Vec3 from rio for the read function signature.
// Arduino's binary.h defines B0–B3 which clash with Eigen.
#undef B0
#undef B1
#undef B2
#undef B3
#include <rio/rio_types.h>

// ── Public API ───────────────────────────────────────────────

/// Call once in setup() after Wire.begin() + Wire.setClock(400000).
/// Powers on the accelerometer, verifies chip IDs, and sets ranges.
/// Returns true on success.
bool bmi085Init();

/// Read accelerometer and gyroscope.
/// acc is in m/s², gyr is in rad/s.
/// Returns false on I2C error (call bmi085Recover() to fix).
bool bmi085Read(rio::Vec3& acc, rio::Vec3& gyr);

/// Attempt to recover from a stuck I2C bus by toggling SCL,
/// then re-initialise the sensor.
void bmi085Recover();

/// Print all detected I2C addresses to Serial (debug helper).
void bmi085ScanI2C();
