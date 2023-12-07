#include <Arduino.h>
#include <Metro.h>
#include <Wire.h>
#include <SPI.h>

#include <SparkFun_u-blox_GNSS_v3.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_LPS2X.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20649.h>

#include <states/State.h>
#include <states/PreLaunch.h>

#define LOOP_RATE 100

Metro timer = Metro(1000 / LOOP_RATE);

State *state = new PreLaunch();


Adafruit_BNO055 * bno;
Adafruit_LPS25 * lps;
SFE_UBLOX_GNSS * gps;
Adafruit_ICM20649 * icm;

void setup()
{
    Serial.begin(115200);

    while(!Serial);
    Serial.println("Beginning Flight Computer");

    bno = new Adafruit_BNO055(-1, 0x28, &Wire);
    lps = new Adafruit_LPS25();
    gps = new SFE_UBLOX_GNSS();
    icm = new Adafruit_ICM20649();
    
    if(!bno->begin())
    {
        Serial.println("BNO Not Detected!");
        while(1);
    } else {
        Serial.println("BNO Detected!");
    }

    if(!lps->begin_I2C()) {
        Serial.println("LPS25 Detected!");
    } else {
        Serial.println("LPS25 Not Detected!");
    }

    if(!gps->begin()) {
        Serial.println("GPS Detected");
    } else {
        Serial.println("GPS Not detected");
    }

    if(!icm->begin_I2C()) {
        Serial.println("ICM Detected");
    } else {
        Serial.println("ICM Not Detected");
    }

    Wire.begin();
    Wire.setClock(400000);

    timer.reset();

    state->initialize();
}

void loop()
{
    if (timer.check() == 1)
    {

        // byte SIV = gps->getSIV();
        // Serial.print("Satellites: "); Serial.println(SIV);

        state->loop();
        State *nextState = state->nextState();
        if (nextState != nullptr)
        {
            delete state;
            state = nextState;
            state->initialize();
        }
    }
};