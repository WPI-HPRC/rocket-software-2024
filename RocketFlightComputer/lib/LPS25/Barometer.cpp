/* Rayhan Semy and Amber Cronin
10/7/2023*/
#include <Barometer.h>
#include <Wire.h>

Barometer::Barometer(int sda, int scl) {
    this->sda = sda;
    this->scl = scl;
}

void Barometer::init()
{
    TwoWire W(sda, scl);
    this->sensor.begin_I2C(LPS2X_I2CADDR_DEFAULT, &W);
}
LPS25_data Barometer::read()
{
    sensors_event_t pressure, temp;
    this->sensor.getEvent(&pressure, &temp);

    LPS25_data data = {
        .pressure = pressure.pressure,
        .temp = temp.temperature,
    };

    return data;
}
