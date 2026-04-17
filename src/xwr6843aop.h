#pragma once
// ──────────────────────────────────────────────────────────────
// xWR6843AOP radar driver for Teensy 4.1
//
// Wiring (TTL 3.3V):
//   Radar GND    <-> Teensy GND
//   Radar CLI_TX ->  Teensy Serial4 RX (pin 7)
//   Radar CLI_RX <-  Teensy Serial4 TX (pin 8)
//   Radar DATA_TX->  Teensy Serial3 RX (pin 15)
//   Radar DATA_RX<-  Teensy Serial3 TX (pin 14)  (optional)
// ──────────────────────────────────────────────────────────────
#include <Arduino.h>

// ── Data types ───────────────────────────────────────────────

/// One detected point parsed from TLV type 1 with axis remapping applied.
/// The raw TLV frame (x=forward, y=left, z=up) is remapped to match the
/// convention used by the rest of the stack (same as the PX4 driver):
///   x =  raw_y   (left)
///   y =  raw_x   (forward)
///   z = -raw_z   (down)
/// vr is the signed radial Doppler velocity (m/s). No normalisation is applied.
struct RadarPoint {
  float x, y, z;  // Cartesian position in remapped radar frame (m)
  float vr;        // radial velocity (m/s)
};

/// Result of one radar frame read + parse cycle.
/// Contains raw points only — normalisation to unit bearing vectors
/// is the responsibility of the consumer (e.g. rio_eskf.cpp).
struct RadarFrame {
  static constexpr uint32_t MAX_POINTS = 128;

  uint32_t   numObj;               // objects reported in frame header
  uint32_t   numTLV;               // TLVs reported in frame header
  uint32_t   numRaw;               // valid points parsed from TLV type 1

  RadarPoint raw[MAX_POINTS];      // raw xyz + vr, radar frame, no filtering

  bool valid;                      // true if a complete frame was read
};

// ── Public API ───────────────────────────────────────────────

/// Call once in setup().  Starts CLI + DATA UARTs, waits for the radar
/// boot banner, then sends the full configuration sequence.
/// Returns true if all config commands were accepted.
bool xwr6843aopInit();

/// Call once per loop() iteration.  Reads one complete frame from the
/// DATA UART and stores all detected points verbatim in frame.raw[].
/// No filtering, no normalisation.  Check frame.valid before using results.
void xwr6843aopUpdate(RadarFrame& frame);

/// Forward any stray CLI bytes to Serial (boot messages, async prints).
void xwr6843aopDrainCli();

/// Print raw xyz + vr + range for every detected point.
/// Use this to verify the physical axis mapping of the sensor.
void xwr6843aopPrintRaw(const RadarFrame& frame);