#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

class BNO055
{
private:
    int pin;
    Adafruit_BNO055 bno;

public:
    BNO055(int num);
    
    Adafruit_BNO055 getbno();
    void initSensor()
    {

        if (getbno().begin())
        {
            Serial.println("BNO055 found");
        }
        else
        {
            Serial.println("BNO055 not found");
            while (1)
                ;
        }
    };
    void readSensor()
    {
        imu::Quaternion quat = bno.getQuat();

        Serial.println("qW: ");
        Serial.println(quat.w(), 4);
        Serial.print(" qX: ");
        Serial.print(quat.y(), 4);
        Serial.print(" qY: ");
        Serial.print(quat.x(), 4);
        Serial.print(" qZ: ");
        Serial.println(quat.z(), 4);
    }
};
