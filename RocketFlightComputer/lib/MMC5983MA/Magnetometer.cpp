#include "Magnetometer.h"

Magnetometer::Magnetometer() : mag() {}

bool Magnetometer::init() {
  if (!this->mag.begin()) {
    return false;
  }

  this->mag.softReset();
  this->mag.setFilterBandwidth(800); // Filter Bandwith - BITS 0 | 0 (100Hz - 0.4 mG RMS Noise)

  return true;
}

MMC_data Magnetometer::read() {
  // uint32_t rawX, rawY, rawZ;
  // this->mag.getMeasurementXYZ(&rawX, &rawY, &rawZ);
  uint32_t rawX = mag.getMeasurementX();
  uint32_t rawY = mag.getMeasurementY();
  uint32_t rawZ = mag.getMeasurementZ();

  // Convert raw 18 bit unsigned integer to +/- 1.0 approximate zero is 2^17 (131072)
  double scaledX = (double) rawX - 131072.0;
  scaledX /= 131072.0;
  double scaledY = (double) rawY - 131072.0;
  scaledY /= 131072.0;
  double scaledZ = (double) rawZ - 131072.0;
  scaledZ /= 131072.0;

  return MMC_data {
    .x = scaledX * 800000.0, // [nT]
    .y = scaledY * 800000.0, // [nT]
    .z = scaledZ * 800000.0  // [nT]
    // .x = ((x - 135594.0f) / 135594.0f) * 800.0f, // [uT]
    // .y = ((y - 134286.0f) / 134286.0f) * 800.0f, // [uT]
    // .z = ((z - 138874.0f) / 138874.0f) * 800.0f  // [uT]
  };
}
