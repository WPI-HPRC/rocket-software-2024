#include "Magnetometer.h"

Magnetometer::Magnetometer() : mag() {}

bool Magnetometer::init() {
  if (!this->mag.init()) {
    return false;
  }

  this->mag.softReset();
  return true;
}
