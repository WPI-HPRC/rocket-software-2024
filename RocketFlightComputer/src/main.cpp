#include <Arduino.h>
#include <Metro.h>
#include <Wire.h>
#include <SPI.h>

#include <SparkFun_u-blox_GNSS_v3.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_LPS2X.h>
#include <SparkFun_MMC5983MA_Arduino_Library.h>
#include <ICM42688.h>

#include <states/State.h>
#include <states/PreLaunch.h>

#include <Barometer.h>
#include <BNO055.h>
#include <GNSS.h>
#include <Accelerometer.h>
#include <Magnetometer.h>

Metro timer = Metro(1000 / LOOP_RATE);

struct Sensors sensors;

State *state = new PreLaunch(&sensors);

void setup()
{
    Serial.begin(115200);

    while(!Serial);
    Serial.println("Beginning Flight Computer");

    Wire.begin();
    Wire.setClock(400000);

    sensors = {
        .barometer = new Barometer(),
        .gnss = new GNSS(),
        .bno055 = new BNO055(0),
        .mag = new Magnetometer(),
        .acc = new Accelerometer(0x68),
    };

    if(!sensors.bno055->init()) {
        Serial.println("[Sensorboard] No BNO055 Detected");
    } else {
        Serial.println("[Sensorboard] BNO055 IMU Detected");
    }

    if(!sensors.barometer->init(0x5C)) {
        Serial.println("[Sensorboard] No LPS25 Detected");
    } else {
        Serial.println("[Sensorboard] LPS25 Barometer Detected");
    }

    if(!sensors.mag->init()) {
        Serial.println("[Sensorboard] No MMC5983 Detected");
    } else {
        Serial.println("[Sensorboard] MMC5983 Detected");
    }

    if(!sensors.acc->init()) {
        Serial.println("[Sensorboard] No ICM42688 Detected");
    } else {
        Serial.println("[Sensorboard] ICM42688 Detected");
    }

    if(!sensors.gnss->init()) {
        Serial.println("[Sensorboard] No NEOM10S Detected");
    } else {
        Serial.println("[Sensorboard] NEOM10S GPS Detected");
    }

    state->initialize();

    timer.reset();
}

uint32_t previousTime = 0;

void loop()
{
    if (timer.check() == 1)
    {
        state->loop();
        previousTime = millis();
        State *nextState = state->nextState();
        if (nextState != nullptr)
        {
            delete state;
            state = nextState;
            state->initialize();
        }
    }
};
