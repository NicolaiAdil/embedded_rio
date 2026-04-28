#include "bmp581.h"
#include <Wire.h>
#include <SparkFun_BMP581_Arduino_Library.h>

#include "config.h"

// SDO → GND → 0x46. SDO → VDDIO → 0x47.
static constexpr uint8_t BMP581_I2C_ADDR = 0x46;

static BMP581 s_baro;

bool bmp581Init() {
  Wire2.begin();
  Wire2.setClock(400000);

  if (s_baro.beginI2C(BMP581_I2C_ADDR, Wire2) != BMP5_OK) {
#if USB_PRINT_ENABLED
    Serial.println("BMP581 beginI2C failed");
#endif
    return false;
  }

  // 50 Hz output, OSR_T ×1, OSR_P ×8 (matches what we were trying manually).
  bmp5_osr_odr_press_config cfg = {0};
  cfg.odr      = BMP5_ODR_50_HZ;
  cfg.press_en = BMP5_ENABLE;
  cfg.osr_t    = BMP5_OVERSAMPLING_1X;
  cfg.osr_p    = BMP5_OVERSAMPLING_8X;
  if (s_baro.setOSRMultipliers(&cfg) != BMP5_OK) {
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

bool bmp581Read(float& temp_c, float& press_pa) {
  bmp5_sensor_data data = {0};
  if (s_baro.getSensorData(&data) != BMP5_OK) return false;
  temp_c   = data.temperature;
  press_pa = data.pressure;
  return true;
}
