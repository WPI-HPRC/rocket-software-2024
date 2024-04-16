#include "Magnetometer.h"

Magnetometer::Magnetometer() : mag() {}

bool Magnetometer::init() {
  if (!this->mag.begin()) {
    return false;
  }

  this->mag.softReset();
  // this->mag.setFilterBandwidth(800); // Filter Bandwith - BITS 0 | 0 (100Hz - 0.4 mG RMS Noise)
  
  mag.setContinuousModeFrequency(50);
  Serial.print("[MMC5983MA] Continuous Frequency: "); Serial.println(mag.getContinuousModeFrequency());

  mag.enableAutomaticSetReset();
  Serial.print("[MMC5983MA] Automatic Set/Reset (?): ");
  Serial.println(mag.isAutomaticSetResetEnabled() ? "enabled" : "disabled");

  mag.enableContinuousMode();
  Serial.print("[MMC5983MA] Continuous Mode (?): ");
  Serial.println(mag.isContinuousModeEnabled() ? "enabled" : "disabled");

  mag.enableInterrupt();
  Serial.print("[MMC5983MA] Interrupt Mode (?): ");
  Serial.println(mag.isInterruptEnabled() ? "enabled" : "disabled");

  newDataAvailable = true;

  return true;
}

MMC_data Magnetometer::read() {

  if(newDataAvailable) {
    newDataAvailable = false;
    mag.clearMeasDoneInterrupt();

    mag.readFieldsXYZ(&rawValX, &rawValY, &rawValZ);

    scaledX = (double) rawValX - 131072.0;
    scaledX /= 131072.0;

    scaledY = (double) rawValY - 131072.0;
    scaledY /= 131072.0;
    
    scaledZ = (double) rawValZ - 131072.0;
    scaledZ /= 131072.0;
  }

  return MMC_data {
    .x = scaledX,
    .y = scaledY,
    .z = scaledZ
  };
}

void Magnetometer::handleInterrupt() {
  this->newDataAvailable = true;
}