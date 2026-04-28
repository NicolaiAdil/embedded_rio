#include "bmp581.h"
#include <SPI.h>

// ──────────────────────────────────────────────────────────────
// Register map
// ──────────────────────────────────────────────────────────────
static constexpr uint8_t REG_CHIP_ID     = 0x01;
static constexpr uint8_t REG_INT_STATUS  = 0x27;  // bit 5 = drdy_data_reg
static constexpr uint8_t REG_TEMP_XLSB  = 0x1D;  // first of 6 data bytes
static constexpr uint8_t REG_OSR_CONFIG  = 0x36;  // oversampling
static constexpr uint8_t REG_ODR_CONFIG  = 0x37;  // ODR + power mode
static constexpr uint8_t REG_CMD         = 0x7E;

// ──────────────────────────────────────────────────────────────
// Constants
// ──────────────────────────────────────────────────────────────
static constexpr uint8_t  CHIP_ID        = 0x50;
static constexpr uint8_t  CMD_SOFT_RESET = 0xB6;

// ODR_CONFIG[6:2] = odr_sel, [1:0] = pwr_mode
// odr_sel 0x0B → 50 Hz  (from Bosch BMP5 API)
// pwr_mode 0x01 → Normal (continuous)
static constexpr uint8_t  ODR_50HZ_NORMAL = (0x0B << 2) | 0x01;

// OSR_CONFIG[5:3] = osr_t, [2:0] = osr_p
// osr_t 0 = ×1, osr_p 3 = ×8 (good balance of noise/speed at 50 Hz)
static constexpr uint8_t  OSR_T1_P8 = (0 << 3) | 3;

static constexpr uint32_t SPI_CLK_HZ = 8000000;  // BMP581 max 10 MHz

// ──────────────────────────────────────────────────────────────
// Module state
// ──────────────────────────────────────────────────────────────
static uint8_t     s_cs          = 10;
static SPISettings s_spi(SPI_CLK_HZ, MSBFIRST, SPI_MODE0);

// ──────────────────────────────────────────────────────────────
// SPI helpers
// ──────────────────────────────────────────────────────────────
static inline void csLow()  { digitalWriteFast(s_cs, LOW);  }
static inline void csHigh() { digitalWriteFast(s_cs, HIGH); }

static void spiWrite(uint8_t reg, uint8_t val) {
  SPI.beginTransaction(s_spi);
  csLow();
  SPI.transfer(reg & 0x7F);  // MSB = 0 → write
  SPI.transfer(val);
  csHigh();
  SPI.endTransaction();
}

static void spiReadBurst(uint8_t reg, uint8_t* buf, uint8_t len) {
  SPI.beginTransaction(s_spi);
  csLow();
  SPI.transfer(reg | 0x80);  // MSB = 1 → read
  for (uint8_t i = 0; i < len; i++) buf[i] = SPI.transfer(0x00);
  csHigh();
  SPI.endTransaction();
}

static uint8_t spiRead(uint8_t reg) {
  uint8_t val;
  spiReadBurst(reg, &val, 1);
  return val;
}

// ──────────────────────────────────────────────────────────────
// Public API
// ──────────────────────────────────────────────────────────────
bool bmp581Init(uint8_t cs_pin) {
  s_cs = cs_pin;
  pinMode(s_cs, OUTPUT);
  csHigh();
  SPI.begin();

  // Soft reset and wait for startup
  spiWrite(REG_CMD, CMD_SOFT_RESET);
  delay(4);  // datasheet: 2 ms start-up after reset

  // Verify chip ID
  const uint8_t id = spiRead(REG_CHIP_ID);
  if (id != CHIP_ID) {
    Serial.print("BMP581 chip ID mismatch: got 0x");
    Serial.print(id, HEX);
    Serial.println(", expected 0x50");
    return false;
  }
  Serial.println("BMP581 chip ID OK (0x50)");

  // Oversampling: temp ×1, pressure ×8
  spiWrite(REG_OSR_CONFIG, OSR_T1_P8);
  delay(2);

  // ODR 50 Hz, normal (continuous) mode
  spiWrite(REG_ODR_CONFIG, ODR_50HZ_NORMAL);
  delay(2);

  Serial.println("BMP581 init OK");
  return true;
}

bool bmp581Read(float& temp_c, float& press_pa) {
  // Check data-ready flag (INT_STATUS bit 5)
  if (!(spiRead(REG_INT_STATUS) & (1 << 5))) return false;

  // Burst-read 6 bytes: temp[xlsb, lsb, msb], press[xlsb, lsb, msb]
  uint8_t buf[6];
  spiReadBurst(REG_TEMP_XLSB, buf, 6);

  // Temperature: 24-bit two's complement, 1 LSB = 1/65536 °C
  int32_t raw_t = ((uint32_t)buf[2] << 16) | ((uint32_t)buf[1] << 8) | buf[0];
  if (raw_t & 0x800000) raw_t |= (int32_t)0xFF000000;  // sign-extend
  temp_c = static_cast<float>(raw_t) / 65536.0f;

  // Pressure: 24-bit unsigned fixed-point, 1 LSB = 1/64 Pa
  const uint32_t raw_p = ((uint32_t)buf[5] << 16) | ((uint32_t)buf[4] << 8) | buf[3];
  press_pa = static_cast<float>(raw_p) / 64.0f;

  return true;
}
