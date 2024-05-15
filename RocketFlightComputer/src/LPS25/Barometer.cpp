/* Rayhan Semy and Amber Cronin
10/7/2023*/
#include "Barometer.h"
#include <Wire.h>

Barometer::Barometer() {}

bool Barometer::init(int addr) {
    if (!this->sensor.begin_I2C(addr)) {
        return false;
    }
    this->sensor.setDataRate(LPS25_RATE_25_HZ);
    return true;
}

LPS25_data Barometer::read() {
    sensors_event_t pressure, temp;
    this->sensor.getEvent(&pressure, &temp);

    LPS25_data data = {
        .pressure = pressure.pressure,
        .temp = temp.temperature,
    };

    return data;
}
