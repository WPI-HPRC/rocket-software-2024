#include "Accelerometer.h"
#include <Wire.h>

Accelerometer::Accelerometer(int addr) : icm42688(Wire, addr) {}

bool Accelerometer::init() {

  if(this->icm42688.begin() != 1) {
    return false;
  }

  this->icm42688.setAccelFS(ICM42688::gpm16);
  this->icm42688.setGyroFS(ICM42688::dps250);

  this->icm42688.setAccelODR(ICM42688::odr100);
  this->icm42688.setGyroODR(ICM42688::odr100);

  return true;
}

ICM_data Accelerometer::read() {
  this->icm42688.getAGT();

  return ICM_data {
    .accX = this->icm42688.accX(),
    .accY = this->icm42688.accY(),
    .accZ = this->icm42688.accZ(),
    .gyroX = this->icm42688.gyrX(),
    .gyroY = this->icm42688.gyrY(),
    .gyroZ = this->icm42688.gyrZ(),
  };
}
