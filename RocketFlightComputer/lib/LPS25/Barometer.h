/* Rayhan Semy and Amber Cronin
10/7/2023*/
#pragma once
#include <Arduino.h>
#include <Adafruit_LPS2X.h>


typedef struct
{
    sensors_event_t temp;
    sensors_event_t pressure;
} LPS25_data;


class Barometer
{
public:
    Barometer();
    void init(int sda, int scl);
    void readSensor(LPS25_data *data);

private:
    Adafruit_LPS25 sensor;
};