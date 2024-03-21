#include "Sensors.h"
#include "Accelerometer.h"
#include "Barometer.h"
#include "Magnetometer.h"
#include "utility.hpp"
#include <Arduino.h>

Utility::SensorPacket Sensors::readSensors() {
  Utility::SensorPacket sensorPacket;

  LPS25_data barometerData = this->barometer->read();

  sensorPacket.pressure = barometerData.pressure; // [hPa/mBar]
  sensorPacket.altitude = Utility::pressureToAltitude(barometerData.pressure); // m

  ICM_data accelerometerData = this->acc->read();

  sensorPacket.accelX = accelerometerData.accX;
  sensorPacket.accelY = accelerometerData.accY;
  sensorPacket.accelZ = accelerometerData.accZ;

  sensorPacket.gyroX = accelerometerData.gyroX;
  sensorPacket.gyroY = accelerometerData.gyroY;
  sensorPacket.gyroZ = accelerometerData.gyroZ;

  MMC_data magnetometerData = this->mag->read();

  sensorPacket.magX = magnetometerData.x;
  sensorPacket.magY = magnetometerData.y;
  sensorPacket.magZ = magnetometerData.z;

  // Serial.print(sensorPacket.magX, 5); Serial.print(",");
  // Serial.print(sensorPacket.magY, 5); Serial.print(",");
  // Serial.println(sensorPacket.magZ, 5);

  // Apply Accelerometer Biases

  //TODO: Make these matrices constants, I tried and it was mad :()
  BLA::Matrix<3,3> accelScaleFactor = {
    1.515094, -0.057311, -0.115285,
    -0.057311, 1.081834, 0.021162,
    -0.115285, 0.021162, 1.002006
  };

  BLA::Matrix<3> accelBias = {-0.043848, 0.071495, 0.018711};

  BLA::Matrix<3> accelVector = {sensorPacket.accelX, sensorPacket.accelY, sensorPacket.accelZ};

  BLA::Matrix<3> accelCal = accelScaleFactor * (accelVector - accelBias);

  // Update sensor packet with the calibrated values
  sensorPacket.accelX = accelCal(0);
  sensorPacket.accelY = accelCal(1);
  sensorPacket.accelZ = accelCal(2);

  

  sensorPacket.gpsLock = gnss->getLockStatus();

  // if(sensorPacket.gpsLock) {
  sensorPacket.gpsLat = gnss->getLatitude();
  sensorPacket.gpsLong = gnss->getLongitude();
  sensorPacket.gpsAltAGL = gnss->getAltAGL();
  sensorPacket.gpsAltMSL = gnss->getAltMSL();
  sensorPacket.satellites = gnss->getSatellites();
  sensorPacket.epochTime = gnss->getEpochTime();
    // sensorPacket.time = gnss->getTime();
  // } else {
  //   sensorPacket.gpsLat = 0;
  //   sensorPacket.gpsLong = 0;
  //   sensorPacket.gpsAltAGL = 0;
  //   sensorPacket.gpsAltMSL = 0;
  //   sensorPacket.satellites = gnss->getSatellites();
  //   sensorPacket.epochTime = gnss->getEpochTime();
  //   // sensorPacket.time = gnss->getTime();
  // }

  return sensorPacket;
}

