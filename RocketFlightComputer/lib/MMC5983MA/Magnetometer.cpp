#include "Magnetometer.h"

Magnetometer::Magnetometer() : mag() {}

bool Magnetometer::init() {
  if (!this->mag.begin()) {
    return false;
  }

  this->mag.softReset();
  this->mag.setFilterBandwidth(400); // Filter Bandwith - BITS 1 | 0 (400Hz - 0.8 mG RMS Noise)
  return true;
}

MMC_data Magnetometer::read() {
  uint32_t x, y, z;
  this->mag.getMeasurementXYZ(&x, &y, &z);
  return MMC_data {
    .x = ((x - 131072.0f) / 131072.0f) * 800.0f, // [uT]
    .y = ((y - 131072.0f) / 131072.0f) * 800.0f, // [uT]
    .z = ((z - 131072.0f) / 131072.0f) * 800.0f  // [uT]
  };
}
