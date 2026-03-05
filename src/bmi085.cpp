#include "bmi085.h"
#include <Wire.h>

// ──────────────────────────────────────────────────────────────
// I2C addresses
// ──────────────────────────────────────────────────────────────
static constexpr uint8_t ACC_ADDR = 0x19;
static constexpr uint8_t GYR_ADDR = 0x69;

// ──────────────────────────────────────────────────────────────
// Register addresses
// ──────────────────────────────────────────────────────────────
static constexpr uint8_t REG_ACC_CHIP_ID   = 0x00;
static constexpr uint8_t REG_ACC_X_LSB     = 0x12;
static constexpr uint8_t REG_ACC_RANGE     = 0x41;
static constexpr uint8_t REG_ACC_PWR_CONF  = 0x7C;
static constexpr uint8_t REG_ACC_PWR_CTRL  = 0x7D;

static constexpr uint8_t REG_GYR_CHIP_ID   = 0x00;
static constexpr uint8_t REG_GYR_X_LSB     = 0x02;
static constexpr uint8_t REG_GYR_RANGE     = 0x0F;

// ──────────────────────────────────────────────────────────────
// Scale factors
// ──────────────────────────────────────────────────────────────
static constexpr uint8_t ACC_RANGE_REG_VAL = 0x02;           // ±8 g
static constexpr float   ACC_RANGE_G       = 8.0f;
static constexpr float   ACC_SCALE = ACC_RANGE_G * 9.80665f / 32768.0f;

static constexpr uint8_t GYR_RANGE_REG_VAL = 0x00;           // ±2000 °/s
static constexpr float   GYR_SENSITIVITY   = 16.384f;        // LSB/°/s
static constexpr float   GYR_SCALE = (1.0f / GYR_SENSITIVITY) * (M_PI / 180.0f);

// ──────────────────────────────────────────────────────────────
// I2C helpers
// ──────────────────────────────────────────────────────────────
static bool i2c_ok = true;

static bool i2cWriteReg(uint8_t dev, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(dev);
  Wire.write(reg);
  Wire.write(val);
  uint8_t err = Wire.endTransmission();
  if (err != 0) { i2c_ok = false; return false; }
  return true;
}

static bool i2cReadReg(uint8_t dev, uint8_t reg, uint8_t& out) {
  Wire.beginTransmission(dev);
  Wire.write(reg);
  uint8_t err = Wire.endTransmission(false);
  if (err != 0) { i2c_ok = false; out = 0; return false; }
  if (Wire.requestFrom(dev, (uint8_t)1) != 1) { i2c_ok = false; out = 0; return false; }
  out = Wire.read();
  return true;
}

static bool i2cReadBurst(uint8_t dev, uint8_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(dev);
  Wire.write(reg);
  uint8_t err = Wire.endTransmission(false);
  if (err != 0) { i2c_ok = false; memset(buf, 0, len); return false; }
  if (Wire.requestFrom(dev, len) != len) { i2c_ok = false; memset(buf, 0, len); return false; }
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

// ──────────────────────────────────────────────────────────────
// Public API
// ──────────────────────────────────────────────────────────────

bool bmi085Init() {
  uint8_t id = 0;

  // --- Accelerometer ---
  if (!i2cWriteReg(ACC_ADDR, REG_ACC_PWR_CTRL, 0x04)) return false;
  delay(5);
  if (!i2cWriteReg(ACC_ADDR, REG_ACC_PWR_CONF, 0x00)) return false;
  delay(50);

  if (!i2cReadReg(ACC_ADDR, REG_ACC_CHIP_ID, id)) return false;
  Serial.print("Accel chip ID: 0x"); Serial.println(id, HEX);
  if (id != 0x1F) return false;

  if (!i2cWriteReg(ACC_ADDR, REG_ACC_RANGE, ACC_RANGE_REG_VAL)) return false;
  delay(2);

  // --- Gyroscope ---
  if (!i2cReadReg(GYR_ADDR, REG_GYR_CHIP_ID, id)) return false;
  Serial.print("Gyro  chip ID: 0x"); Serial.println(id, HEX);
  if (id != 0x0F) return false;

  if (!i2cWriteReg(GYR_ADDR, REG_GYR_RANGE, GYR_RANGE_REG_VAL)) return false;
  delay(2);

  i2c_ok = true;
  return true;
}

bool bmi085Read(rio::Vec3& acc, rio::Vec3& gyr) {
  uint8_t buf[6];

  if (!i2cReadBurst(ACC_ADDR, REG_ACC_X_LSB, buf, 6)) return false;
  acc.x() = (int16_t)(buf[0] | (buf[1] << 8)) * ACC_SCALE;
  acc.y() = (int16_t)(buf[2] | (buf[3] << 8)) * ACC_SCALE;
  acc.z() = (int16_t)(buf[4] | (buf[5] << 8)) * ACC_SCALE;

  if (!i2cReadBurst(GYR_ADDR, REG_GYR_X_LSB, buf, 6)) return false;
  gyr.x() = (int16_t)(buf[0] | (buf[1] << 8)) * GYR_SCALE;
  gyr.y() = (int16_t)(buf[2] | (buf[3] << 8)) * GYR_SCALE;
  gyr.z() = (int16_t)(buf[4] | (buf[5] << 8)) * GYR_SCALE;

  return true;
}

void bmi085Recover() {
  Serial.println("I2C error — recovering...");

  Wire.end();
  pinMode(19, OUTPUT);
  for (int i = 0; i < 16; i++) {
    digitalWriteFast(19, LOW);  delayMicroseconds(5);
    digitalWriteFast(19, HIGH); delayMicroseconds(5);
  }
  pinMode(19, INPUT);

  Wire.begin();
  Wire.setClock(400000);
  delay(10);

  if (bmi085Init()) Serial.println("Recovery OK");
  else              Serial.println("Recovery FAILED");
}

void bmi085ScanI2C() {
  Serial.println("Scanning I2C bus...");
  int n = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.print("  0x"); Serial.println(a, HEX);
      n++;
    }
  }
  Serial.print("  Total: "); Serial.print(n); Serial.println(" device(s)");
  Serial.println("──────────────────────────────────");
}
