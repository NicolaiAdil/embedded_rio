#pragma once
// BMP581 on I2C (Wire2).
//   P1: VDD, VDDIO → 3.3 V; GND → GND
//   P2: CSB → 3.3 V (force I2C); SDO → GND (addr 0x46);
//       SCL → pin 24; SDA → pin 25
// Pull-ups: 4.7 kΩ from SDA and SCL to 3.3 V (lower for long wires).
#include <Arduino.h>

bool bmp581Init();
bool bmp581Read(float& temp_c, float& press_pa);
