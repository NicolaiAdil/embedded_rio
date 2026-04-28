#pragma once
// ──────────────────────────────────────────────────────────────
// BMP581 barometer driver for Teensy 4.1
// SPI (hardware SPI0), 4-wire.
//
// Wiring (from shuttle board 3.0 → Teensy 4.1):
//   P2·1 CS  → Pin 10  (any digital, passed to bmp581Init)
//   P2·2 SCK → Pin 13  (SPI0 SCK)
//   P2·3 SDO → Pin 12  (SPI0 MISO)
//   P2·4 SDI → Pin 11  (SPI0 MOSI)
//   P1·1/2   → 3.3 V
//   P1·3     → GND
// ──────────────────────────────────────────────────────────────
#include <Arduino.h>

/// Call once in setup() after SPI.begin() (or let bmp581Init call it).
/// cs_pin: the chip-select pin number (default 10).
/// Returns true on success (chip ID matched).
bool bmp581Init(uint8_t cs_pin = 10);

/// Read latest temperature and pressure.
/// temp_c   : output temperature in °C
/// press_pa : output pressure in Pa
/// Returns false if data-ready flag is not set.
bool bmp581Read(float& temp_c, float& press_pa);
