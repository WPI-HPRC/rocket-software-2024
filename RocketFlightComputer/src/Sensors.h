#include "GNSS.h"
#include "BNO055.h"
#include "Barometer.h"

struct Sensors
{
    Barometer *barometer;
    GNSS *gnss;
    BNO055 *bno055;
};