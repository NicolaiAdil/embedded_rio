#pragma once

// Set to 0 when flying (no USB connected) to skip all Serial prints.
// Set to 1 when debugging on the bench with USB attached.
#define USB_PRINT_ENABLED 1

// ── SD card logging ──────────────────────────────────────────────────────────
// Master switch: set to 1 to enable CSV logging of sensor data to SD card.
#define SD_LOG_ENABLED 0

// Hardware: 1 = Teensy 4.1 built-in SDIO (BUILTIN_SDCARD)
//           0 = Teensy 4.0 external SPI (specify CS pin below)
// Overridden per-environment in platformio.ini; fallback to built-in.
#ifndef SD_BUILTIN
#define SD_BUILTIN 1
#endif
#define SD_CS_PIN  10   // SPI chip-select pin for external SD (Teensy 4.0)

// Per-topic max log rate in Hz. 0 means log every sample.
#define SD_LOG_IMU_HZ   0
#define SD_LOG_RADAR_HZ 0
#define SD_LOG_BARO_HZ  0

// How often to flush buffered data to the SD card (seconds).
#define SD_LOG_FLUSH_INTERVAL_S 1
