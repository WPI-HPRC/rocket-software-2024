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

  /* Apply Accelerometer Biases */
  //TODO: Make these matrices constants, I tried and it was mad :(
  // BLA::Matrix<3,3> accelScaleFactor = {
  //   1.515094, -0.057311, -0.115285,
  //   -0.057311, 1.081834, 0.021162,
  //   -0.115285, 0.021162, 1.002006
  // };

  // BLA::Matrix<3> accelBias = {-0.043848, 0.071495, 0.018711};

  // BLA::Matrix<3> accelVector = {sensorPacket.accelX, sensorPacket.accelY, sensorPacket.accelZ};

  // BLA::Matrix<3> accelCal = accelScaleFactor * (accelVector - accelBias);

  /* Apply Magnetometer Calibration */

  BLA::Matrix<3,3> softIronCal = {
     1.120602,    -0.003242,   0.005510,
    -0.003242,     1.143276,   0.013794,
     0.005510,     0.013794,   1.104641,
  };

  BLA::Matrix<3> hardIronCal = {54062.849827, 5545.343210, 89181.770655};

  BLA::Matrix<3> magVector = {sensorPacket.magX, sensorPacket.magY, sensorPacket.magZ};

  BLA::Matrix<3> magCal = softIronCal * (magVector - hardIronCal);

  // Update sensor packet with the calibrated values
  // sensorPacket.accelX = accelCal(0);
  // sensorPacket.accelY = accelCal(1);
  // sensorPacket.accelZ = accelCal(2);

  sensorPacket.magX = magCal(0);
  sensorPacket.magY = magCal(1);
  sensorPacket.magZ = magCal(2);

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