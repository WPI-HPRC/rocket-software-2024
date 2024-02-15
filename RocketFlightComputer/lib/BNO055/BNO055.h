#pragma once
#include <Arduino.h>
#include <Adafruit_BNO055.h>

// TODO: This class probably needs some additional methods to make mocking and usage easier

class BNO055 {
    private:
        Adafruit_BNO055 bno;
    public:
        BNO055(int32_t sensorID);
        bool init();
        imu::Quaternion read();
};
