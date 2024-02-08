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

  sensorPacket.magX = magnetometerData.x - 134960.0;
  sensorPacket.magY /= 134960.0;
  sensorPacket.magZ = magnetometerData.y - 132340.0;
  sensorPacket.magX /= 132340.0;
  sensorPacket.magY = magnetometerData.z - 138560.0;
  sensorPacket.magZ /= 138560.0;

  sensorPacket.magX *= 8; // [Gauss]-
  sensorPacket.magY *= 8; // [Gauss]
  sensorPacket.magZ *= 8; // [Gauss]

  sensorPacket.gpsLock = gnss->getLockStatus();

  if(sensorPacket.gpsLock) {
    sensorPacket.gpsLat = gnss->getLatitude() / pow(10,7);
    sensorPacket.gpsLong = gnss->getLongitude() / pow(10,7);
    sensorPacket.gpsAltAGL = gnss->getAltAGL() / pow(10,3);
    sensorPacket.gpsAltMSL = gnss->getAltMSL() / pow(10,3);
    sensorPacket.satellites = gnss->getSatellites();
  } else {
    sensorPacket.gpsLat = 0;
    sensorPacket.gpsLong = 0;
    sensorPacket.gpsAltAGL = 0;
    sensorPacket.gpsAltMSL = 0;
    sensorPacket.satellites = gnss->getSatellites();
  }

  return sensorPacket;
}

