#pragma once

// Set to 0 when flying (no USB connected) to skip all Serial prints.
// Set to 1 when debugging on the bench with USB attached.
#define USB_PRINT_ENABLED 0

// Set to 1 to log timestamped IMU + radar data to the Teensy 4.1 SD card.
// Format: CSV with 'I' and 'R' rows, replayed offline via scripts/replay_log.py.
// Flush happens once per second in the rate-logging block.
#define SD_LOG_ENABLED 0
