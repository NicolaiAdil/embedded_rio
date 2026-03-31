#pragma once
// ──────────────────────────────────────────────────────────────
// BMI08x IMU driver for Teensy 4.1
// Supports BMI085 and BMI088 on Bosch Shuttle Board 3.0
//
// Wiring (I2C, SDO1/SDO2 → VDDIO):
//   Accel address: 0x19
//   Gyro  address: 0x69
//   SDA: pin 18
//   SCL: pin 19
//   I2C clock: 400 kHz
// ──────────────────────────────────────────────────────────────
#include <Arduino.h>
#undef B0
#undef B1
#undef B2
#undef B3
#include <rio/rio_types.h>

enum class ImuType { BMI085, BMI088 };

// ── Public API ───────────────────────────────────────────────
/// Call once in setup() after Wire.begin() + Wire.setClock(400000).
/// Powers on the accelerometer, verifies chip IDs, and sets ranges.
/// Returns true on success.
bool bmi08xInit(ImuType type);

/// Read accelerometer and gyroscope.
/// acc is in m/s², gyr is in rad/s.
/// Returns false on I2C error (call bmi08xRecover() to fix).
bool bmi08xRead(rio::Vec3& acc, rio::Vec3& gyr);

/// Attempt to recover from a stuck I2C bus by toggling SCL,
/// then re-initialise the sensor.
void bmi08xRecover();

/// Print all detected I2C addresses to Serial (debug helper).
void bmi08xScanI2C();