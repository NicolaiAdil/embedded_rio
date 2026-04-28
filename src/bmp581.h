#pragma once
// ──────────────────────────────────────────────────────────────
// BMP581 barometer driver for Teensy 4.1 — I2C on Wire2.
//
// Wiring (shuttle board 3.0 → Teensy 4.1):
//   P1·1 VDD   → 3.3 V
//   P1·2 VDDIO → 3.3 V
//   P1·3 GND   → GND
//   P2·1 CSB   → 3.3 V          (forces I2C mode)
//   P2·2 SCL   → Pin 24         (Wire2 SCL)
//   P2·3 SDO   → GND            (selects I2C addr 0x46)
//   P2·4 SDA   → Pin 25         (Wire2 SDA)
//
// Pull-ups: 4.7 kΩ from SDA and SCL to 3.3 V (lower if wires are long).
// ──────────────────────────────────────────────────────────────
#include <Arduino.h>

/// Call once in setup(). Initialises Wire2 internally.
/// Returns true on success (chip ID matched).
bool bmp581Init();

/// Read latest temperature and pressure.
/// temp_c   : output temperature in °C
/// press_pa : output pressure in Pa
/// Returns false if data-ready flag is not set.
bool bmp581Read(float& temp_c, float& press_pa);
