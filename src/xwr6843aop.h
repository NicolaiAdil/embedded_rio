#pragma once
// ──────────────────────────────────────────────────────────────
// xWR6843AOP radar driver for Teensy 4.1
//
// Wiring (TTL 3.3V):
//   Radar GND    <-> Teensy GND
//   Radar CLI_TX ->  Teensy Serial2 RX (pin 7)
//   Radar CLI_RX <-  Teensy Serial2 TX (pin 8)
//   Radar DATA_TX->  Teensy Serial3 RX (pin 15)
//   Radar DATA_RX<-  Teensy Serial3 TX (pin 14)  (optional)
// ──────────────────────────────────────────────────────────────

#include <Arduino.h>
#include <rio/rio_eskf.h>   // rio::RadarDoppler, rio::Vec3

// ── Data types ───────────────────────────────────────────────

/// Result of one radar frame read + parse cycle.
/// Measurements are produced as rio::RadarDoppler, ready for eskf.correct().
struct RadarFrame {
  static constexpr uint32_t MAX_POINTS = 128;

  uint32_t           numObj;                     // objects reported in header
  uint32_t           numTLV;                     // TLVs reported in header
  uint32_t           numMeas;                    // valid Doppler measurements parsed
  rio::RadarDoppler  meas[MAX_POINTS];           // ready for eskf.correct()
  bool               valid;                      // true if a frame was successfully read
};

// ── Public API ───────────────────────────────────────────────

/// Call once in setup().  Starts CLI + DATA UARTs, waits for the
/// radar boot banner, then sends the full configuration.
/// Returns true if all config commands were accepted.
bool xwr6843aopInit();

/// Call once per loop() iteration.  Attempts to read one complete
/// frame from the DATA UART and parse TLV type 1 (detected points)
/// into rio::RadarDoppler measurements.
/// Populates `frame`; check `frame.valid` to see if a frame arrived.
void xwr6843aopUpdate(RadarFrame& frame);

/// Forward any stray CLI bytes to Serial (boot messages, async prints).
void xwr6843aopDrainCli();

/// Print a parsed frame to Serial in human-readable form.
void xwr6843aopPrintFrame(const RadarFrame& frame);
