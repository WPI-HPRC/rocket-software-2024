#pragma once
#include "GNSS.h"
#include "BNO055.h"
#include "Barometer.h"
#include "Magnetometer.h"
#include "Accelerometer.h"

struct Sensors {
    Barometer *barometer;
    GNSS *gnss;
    BNO055 *bno055;
    Magnetometer *mag;
    Accelerometer *acc;
};
