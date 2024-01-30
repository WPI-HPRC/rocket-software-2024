#include "Accelerometer.h"
#include <Wire.h>

Accelerometer::Accelerometer(int addr) : icm2688(Wire, addr) {}

void Accelerometer::init() {
  this->icm2688.setAccelFS(ICM42688::gpm16);
  this->icm2688.setGyroFS(ICM42688::dps250);

  this->icm2688.setAccelODR(ICM42688::odr100);
  this->icm2688.setGyroODR(ICM42688::odr100);
}
