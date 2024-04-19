#include "Sensors.h"
#include "utility.hpp"
#include <Arduino.h>

Utility::SensorPacket Sensors::readSensors() {
  Utility::SensorPacket sensorPacket;

  sensorPacket.timestamp = millis();

  LPS25_data barometerData = this->barometer->read();

  sensorPacket.pressure = barometerData.pressure; // [hPa/mBar]

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

  sensorPacket.gpsLock = gnss->getLockStatus();

  sensorPacket.gpsLat = gnss->getLatitude();
  sensorPacket.gpsLong = gnss->getLongitude();
  sensorPacket.gpsAltAGL = gnss->getAltAGL();
  sensorPacket.gpsAltMSL = gnss->getAltMSL();
  sensorPacket.satellites = gnss->getSatellites();
  sensorPacket.epochTime = gnss->getEpochTime();

  return sensorPacket;
}
