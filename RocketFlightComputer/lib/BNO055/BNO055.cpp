#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <BNO055.h>

BNO055::BNO055(int32_t sensorID)
{
    bno = Adafruit_BNO055(sensorID);
}

bool BNO055::init() {
    // bno.begin(OPERATION_MODE_ACCGYRO); // TODO/FIXME: confirm that this is the correct mode
    if(!bno.begin()) {
        return false;
    } else {
        return true;
    }
}

imu::Quaternion BNO055::read() {
    return bno.getQuat();
}
