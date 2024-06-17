#pragma once

#include "utility.hpp"
#ifndef PIO_UNIT_TESTING
#include "GPS/GNSS.h"
#include "BNO055/BNO055.h"
#include "LPS25/Barometer.h"
#include "MMC5983MA/Magnetometer.h"
#include "ICM42688/Accelerometer.h"

struct Sensors {
    Barometer *barometer;
    GNSS *gnss;
    // BNO055 *bno055;
    Magnetometer *mag;
    Accelerometer *acc;

    Utility::SensorPacket readSensors();
};
#else
struct Sensors {
    // virtual Utility::SensorPacket readSensors() { Utility::SensorPacket packet; return packet; }
    virtual Utility::SensorPacket readSensors() = 0;
};
#endif
