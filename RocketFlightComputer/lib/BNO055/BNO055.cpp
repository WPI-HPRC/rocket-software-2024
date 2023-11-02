#include <Arduino.h>
#include <Metro.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <BNO055.h>

BNO055::BNO055(int num)
{
    pin = num;
    bno = Adafruit_BNO055(pin);
}
Adafruit_BNO055 BNO055::getbno()
{
    return bno;
}