/* Rayhan Semy and Amber Cronin
10/7/2023*/
#pragma once
#include <Arduino.h>
#include <Adafruit_LPS2X.h>


typedef struct
{
    float pressure;
    float temp;
} LPS25_data;


class Barometer
{
public:
    Barometer(int sda, int scl);
    void init();
    LPS25_data read();

private:
    Adafruit_LPS25 sensor;
    int sda;
    int scl;
};
