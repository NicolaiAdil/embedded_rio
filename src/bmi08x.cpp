#include "bmi08x.h"
#include <Wire.h>

// ──────────────────────────────────────────────────────────────
// I2C addresses
// ──────────────────────────────────────────────────────────────
static constexpr uint8_t ACC_ADDR = 0x19;
static constexpr uint8_t GYR_ADDR = 0x69;

// ──────────────────────────────────────────────────────────────
// Register addresses
// ──────────────────────────────────────────────────────────────
static constexpr uint8_t REG_ACC_CHIP_ID  = 0x00;
static constexpr uint8_t REG_ACC_X_LSB    = 0x12;
static constexpr uint8_t REG_ACC_CONF     = 0x40;  // ODR and bandwidth
static constexpr uint8_t REG_ACC_RANGE    = 0x41;
static constexpr uint8_t REG_ACC_PWR_CONF = 0x7C;
static constexpr uint8_t REG_ACC_PWR_CTRL = 0x7D;

static constexpr uint8_t REG_GYR_CHIP_ID   = 0x00;
static constexpr uint8_t REG_GYR_X_LSB     = 0x02;
static constexpr uint8_t REG_GYR_RANGE     = 0x0F;
static constexpr uint8_t REG_GYR_BANDWIDTH = 0x10;  // ODR + filter bandwidth

// ──────────────────────────────────────────────────────────────
// Chip IDs
// ──────────────────────────────────────────────────────────────
static constexpr uint8_t CHIP_ID_ACC_BMI085 = 0x1F;
static constexpr uint8_t CHIP_ID_ACC_BMI088 = 0x1E;
static constexpr uint8_t CHIP_ID_GYR        = 0x0F;  // same for both

// ──────────────────────────────────────────────────────────────
// Scale factors — BMI085: ±8 g  (REG_ACC_RANGE = 0x02)
// Scale factors — BMI088: ±12 g (REG_ACC_RANGE = 0x02)
// Gyro ±2000 °/s — identical for both
// ──────────────────────────────────────────────────────────────
static constexpr uint8_t ACC_RANGE_REG_BMI085 = 0x02;
static constexpr float   ACC_SCALE_BMI085     = 8.0f  * 9.80665f / 32768.0f;

static constexpr uint8_t ACC_RANGE_REG_BMI088 = 0x02;
static constexpr float   ACC_SCALE_BMI088     = 12.0f * 9.80665f / 32768.0f;

static constexpr uint8_t GYR_RANGE_REG_VAL = 0x00;
static constexpr float   GYR_SCALE         = (1.0f / 16.384f) * (M_PI / 180.0f);

// ──────────────────────────────────────────────────────────────
// Module state
// ──────────────────────────────────────────────────────────────
static bool    s_i2c_ok    = true;
static ImuType s_imu_type  = ImuType::BMI085;
static float   s_acc_scale = ACC_SCALE_BMI085;

// ──────────────────────────────────────────────────────────────
// I2C helpers
// ──────────────────────────────────────────────────────────────
static bool i2cWriteReg(uint8_t dev, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(dev);
  Wire.write(reg);
  Wire.write(val);
  uint8_t err = Wire.endTransmission();
  if (err != 0) { s_i2c_ok = false; return false; }
  return true;
}

static bool i2cReadReg(uint8_t dev, uint8_t reg, uint8_t& out) {
  Wire.beginTransmission(dev);
  Wire.write(reg);
  uint8_t err = Wire.endTransmission(false);
  if (err != 0) { s_i2c_ok = false; out = 0; return false; }
  if (Wire.requestFrom(dev, (uint8_t)1) != 1) { s_i2c_ok = false; out = 0; return false; }
  out = Wire.read();
  return true;
}

static bool i2cReadBurst(uint8_t dev, uint8_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(dev);
  Wire.write(reg);
  uint8_t err = Wire.endTransmission(false);
  if (err != 0) { s_i2c_ok = false; memset(buf, 0, len); return false; }
  if (Wire.requestFrom(dev, len) != len) { s_i2c_ok = false; memset(buf, 0, len); return false; }
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

// ──────────────────────────────────────────────────────────────
// Public API
// ──────────────────────────────────────────────────────────────
bool bmi08xInit(ImuType type) {
  s_imu_type = type;
  uint8_t id = 0;

  const bool is088        = (type == ImuType::BMI088);
  const uint8_t acc_range = is088 ? ACC_RANGE_REG_BMI088 : ACC_RANGE_REG_BMI085;
  s_acc_scale             = is088 ? ACC_SCALE_BMI088     : ACC_SCALE_BMI085;
  const uint8_t exp_acc_id = is088 ? CHIP_ID_ACC_BMI088  : CHIP_ID_ACC_BMI085;

  // --- Accelerometer power-on (same sequence for both parts) ---
  if (!i2cWriteReg(ACC_ADDR, REG_ACC_PWR_CTRL, 0x04)) return false;
  delay(5);
  if (!i2cWriteReg(ACC_ADDR, REG_ACC_PWR_CONF, 0x00)) return false;
  delay(50);

  if (!i2cReadReg(ACC_ADDR, REG_ACC_CHIP_ID, id)) return false;
  Serial.print("Accel chip ID: 0x"); Serial.println(id, HEX);
  if (id != exp_acc_id) {
    Serial.print("Expected 0x"); Serial.print(exp_acc_id, HEX);
    Serial.println(" — init failed");
    return false;
  }

  if (!i2cWriteReg(ACC_ADDR, REG_ACC_RANGE, acc_range)) return false;
  delay(2);

  // 200 Hz ODR, normal bandwidth (acc_bwp=0xA, acc_odr=0x9).
  // Without this the BMI088 defaults to 100 Hz, causing the same accelerometer
  // reading to be processed multiple times per poll cycle, which inflates the
  // velocity process noise and produces velocity oscillations at rest.
  if (!i2cWriteReg(ACC_ADDR, REG_ACC_CONF, 0xA9)) return false;
  delay(2);

  // --- Gyroscope (chip ID identical for both parts) ---
  if (!i2cReadReg(GYR_ADDR, REG_GYR_CHIP_ID, id)) return false;
  Serial.print("Gyro  chip ID: 0x"); Serial.println(id, HEX);
  if (id != CHIP_ID_GYR) {
    Serial.println("Gyro chip ID mismatch — init failed");
    return false;
  }

  if (!i2cWriteReg(GYR_ADDR, REG_GYR_RANGE, GYR_RANGE_REG_VAL)) return false;
  delay(2);

  // 200 Hz ODR, 64 Hz filter bandwidth.
  // Default is 0x00 (2000 Hz, 532 Hz BW). Options at 200 Hz:
  //   0x04 = 200 Hz ODR, 23 Hz BW  (very smooth, high latency)
  //   0x06 = 200 Hz ODR, 64 Hz BW  (good balance for drone dynamics)
  if (!i2cWriteReg(GYR_ADDR, REG_GYR_BANDWIDTH, 0x06)) return false;
  delay(2);

  s_i2c_ok = true;
  Serial.print("BMI08"); Serial.print(is088 ? '8' : '5');
  Serial.println(" init OK");
  return true;
}

bool bmi08xRead(rio::Vec3& acc, rio::Vec3& gyr) {
  uint8_t buf[6];

  if (!i2cReadBurst(ACC_ADDR, REG_ACC_X_LSB, buf, 6)) return false;
  acc.x() = (int16_t)(buf[0] | (buf[1] << 8)) * s_acc_scale;
  acc.y() = (int16_t)(buf[2] | (buf[3] << 8)) * s_acc_scale;
  acc.z() = (int16_t)(buf[4] | (buf[5] << 8)) * s_acc_scale;

  if (!i2cReadBurst(GYR_ADDR, REG_GYR_X_LSB, buf, 6)) return false;
  gyr.x() = (int16_t)(buf[0] | (buf[1] << 8)) * GYR_SCALE;
  gyr.y() = (int16_t)(buf[2] | (buf[3] << 8)) * GYR_SCALE;
  gyr.z() = (int16_t)(buf[4] | (buf[5] << 8)) * GYR_SCALE;

  return true;
}

void bmi08xRecover() {
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

  if (bmi08xInit(s_imu_type)) Serial.println("Recovery OK");
  else                        Serial.println("Recovery FAILED");
}

void bmi08xScanI2C() {
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