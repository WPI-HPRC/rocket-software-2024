/* Rayhan Semy and Amber Cronin
10/7/2023*/
#include <Barometer.h>
#include <Wire.h>

Barometer::Barometer()
{
}
void Barometer::init(int sda, int scl)
{
    TwoWire W(sda, scl);
    this->sensor.begin_I2C(LPS2X_I2CADDR_DEFAULT, &W);
}
void Barometer::readSensor(LPS25_data *data)
{
    this->sensor.getEvent(&data->pressure, &data->temp);
}