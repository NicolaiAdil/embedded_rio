#include "bmp581.h"
#include <Wire.h>
#include <SparkFun_BMP581_Arduino_Library.h>

#include "config.h"

// SDO → GND → 0x46. SDO → VDDIO → 0x47.
static constexpr uint8_t BMP581_I2C_ADDR = 0x46;

static BMP581 s_baro;

// 100 kHz tolerates long wires + weak pull-ups; 400 kHz was edge-marginal
// and one corrupt bit could leave the BMP581 holding SDA low forever.
static constexpr uint32_t BMP581_I2C_CLOCK_HZ = 100000;

// Wire2 SCL pin on Teensy 4.1 (used for manual bit-bang recovery).
static constexpr uint8_t BMP581_SCL_PIN = 24;

static bool bmp581ApplyConfig() {
  bmp5_osr_odr_press_config cfg = {0};
  cfg.odr      = BMP5_ODR_50_HZ;
  cfg.press_en = BMP5_ENABLE;
  cfg.osr_t    = BMP5_OVERSAMPLING_1X;
  cfg.osr_p    = BMP5_OVERSAMPLING_8X;
  return s_baro.setOSRMultipliers(&cfg) == BMP5_OK;
}

bool bmp581Init() {
  Wire2.begin();
  Wire2.setClock(BMP581_I2C_CLOCK_HZ);

  if (s_baro.beginI2C(BMP581_I2C_ADDR, Wire2) != BMP5_OK) {
#if USB_PRINT_ENABLED
    Serial.println("BMP581 beginI2C failed");
#endif
    return false;
  }

  if (!bmp581ApplyConfig()) {
#if USB_PRINT_ENABLED
    Serial.println("BMP581 setOSRMultipliers failed");
#endif
    return false;
  }

#if USB_PRINT_ENABLED
  Serial.println("BMP581 init OK");
#endif
  return true;
}

void bmp581Recover() {
#if USB_PRINT_ENABLED
  Serial.println("BMP581 / Wire2 I2C error — recovering...");
#endif

  Wire2.end();
  pinMode(BMP581_SCL_PIN, OUTPUT);
  for (int i = 0; i < 16; i++) {
    digitalWriteFast(BMP581_SCL_PIN, LOW);  delayMicroseconds(5);
    digitalWriteFast(BMP581_SCL_PIN, HIGH); delayMicroseconds(5);
  }
  pinMode(BMP581_SCL_PIN, INPUT);

  Wire2.begin();
  Wire2.setClock(BMP581_I2C_CLOCK_HZ);
  delay(10);

  if (s_baro.beginI2C(BMP581_I2C_ADDR, Wire2) != BMP5_OK) {
#if USB_PRINT_ENABLED
    Serial.println("BMP581 recovery FAILED (beginI2C)");
#endif
    return;
  }
  bmp581ApplyConfig();
#if USB_PRINT_ENABLED
  Serial.println("BMP581 recovery OK");
#endif
}

bool bmp581Read(float& temp_c, float& press_pa) {
  bmp5_sensor_data data = {0};
  if (s_baro.getSensorData(&data) != BMP5_OK) {
    bmp581Recover();
    return false;
  }
  temp_c   = data.temperature;
  press_pa = data.pressure;
  return true;
}
