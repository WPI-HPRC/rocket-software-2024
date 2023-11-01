#include <Arduino.h>
#include <lib/MetroTimer/Metro.h>
#include <Wire.h>
#include <SPI.h>

#include <SparkFun_u-blox_GNSS_v3.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_LPS2X.h>

#include <states/State.h>
#include <states/PreLaunch.h>

#define LOOP_RATE 100

Metro timer = Metro(1000 / LOOP_RATE);
int counter = 0;
uint32_t timestamp;

long loopStartTime;
long loopTime;
long previousTime;

State *state = new PreLaunch();

void setup()
{
    Serial.begin(115200);

    Wire.begin();
    Wire.setClock(400000);

    timer.reset();
    previousTime = millis();
    loopStartTime = millis();

    state = LOOP;
};

void readSensors(){

};

void loop()
{

    if (timer.check() == 1)
    {
        state->loop();
        State *nextState = state->nextState();
        if (nextState != nullptr)
        {
            state = nextState;
            state->initialize();
        }

        counter++;
    }
};