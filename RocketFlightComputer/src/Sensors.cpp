#include "Sensors.h"
#include "utility.hpp"
#include <Arduino.h>

// Sensor Calbiration Factors
float biasAccelX = 0;
float biasAccelY = 0.025;
float biasAccelZ = 0.02;

float biasGyroX = 0.0017;
float biasGyroY = -0.0054;
float biasGyroZ = -0.0046;

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

  sensorPacket.magX = magnetometerData.x - 135180.0;
  sensorPacket.magX /= 135180.0;
  sensorPacket.magY = magnetometerData.y - 132860.0;
  sensorPacket.magY /= 132860.0;
  sensorPacket.magZ = magnetometerData.z - 138400.0;
  sensorPacket.magZ /= 138400.0;

  // sensorPacket.magX = magnetometerData.x - 134960.0;
  // sensorPacket.magY /= 134960.0;
  // sensorPacket.magZ = magnetometerData.y - 132340.0;
  // sensorPacket.magX /= 132340.0;
  // sensorPacket.magY = magnetometerData.z - 138560.0;
  // sensorPacket.magZ /= 138560.0;

  sensorPacket.magX *= 8; // [T]
  sensorPacket.magY *= 8; // [T]
  sensorPacket.magZ *= 8; // [T]

  sensorPacket.gpsLock = gnss->getLockStatus();

  if(sensorPacket.gpsLock) {
    sensorPacket.gpsLat = gnss->getLatitude();
    sensorPacket.gpsLong = gnss->getLongitude();
    sensorPacket.gpsAltAGL = gnss->getAltAGL();
    sensorPacket.gpsAltMSL = gnss->getAltMSL();
    sensorPacket.satellites = gnss->getSatellites();
    sensorPacket.epochTime = gnss->getEpochTime();
    // sensorPacket.time = gnss->getTime();
  } else {
    sensorPacket.gpsLat = 0;
    sensorPacket.gpsLong = 0;
    sensorPacket.gpsAltAGL = 0;
    sensorPacket.gpsAltMSL = 0;
    sensorPacket.satellites = gnss->getSatellites();
    sensorPacket.epochTime = gnss->getEpochTime();
    // sensorPacket.time = gnss->getTime();
  }

  return sensorPacket;
}

