#include "EKF/EKF.h"
#include "utility.hpp"
#include <Arduino.h>
#include <Metro.h>
#include <Wire.h>
#include <SPI.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_LPS2X.h>
#include <SparkFun_MMC5983MA_Arduino_Library.h>
#include <ICM42688.h>

#include <states/State.h>
#include <states/00-PreLaunch.h>

#include <Sensors.h>

bool sdCardInitialized = false;
fs::File dataFile;

Metro timer = Metro(1000 / LOOP_RATE);

struct Sensors sensors;
StateEstimator *stateEstimator = new StateEstimator();
// Start in pre-launch
State *state = new PreLaunch(&sensors, stateEstimator);

void setup()
{
    Serial.begin(115200);
#ifdef WAIT_FOR_SERIAL
    while (!Serial) {}
#endif
    Serial.println("Beginning Flight Computer");

    Wire.begin();
    Wire.setClock(400000);

    SPI.setSCK(18);
    SPI.setTX(19);
    SPI.setRX(16);
    SPI.begin();
    SPI.beginTransaction(SPISettings(6000000, MSBFIRST, SPI_MODE0));

    // Initialize all sensors
    sensors = {
        .barometer = new Barometer(),
        .gnss = new GNSS(),
        .bno055 = new BNO055(0),
        .mag = new Magnetometer(),
        .acc = new Accelerometer(0x68),
    };
    pinMode(SERVO_FEEDBACK_GPIO, INPUT);

    if(!sensors.bno055->init()) {
        Serial.println("[Sensorboard] No BNO055 Detected");
    } else {
        Serial.println("[Sensorboard] BNO055 IMU Detected");
    }

    if (!sensors.barometer->init(0x5C))
    {
        Serial.println("[Sensorboard] No LPS25 Detected");
    }
    else
    {
        Serial.println("[Sensorboard] LPS25 Barometer Detected");
    }

    if (!sensors.mag->init())
    {
        Serial.println("[Sensorboard] No MMC5983 Detected");
    }
    else
    {
        Serial.println("[Sensorboard] MMC5983 Detected");
    }

    if(!sensors.acc->init()) {
        Serial.println("[Sensorboard] No ICM42688 Detected");
    } else {
        Serial.println("[Sensorboard] ICM42688 Detected");
    }

    if (!sensors.gnss->init())
    {
        Serial.println("[Sensorboard] No NEOM10S Detected");
    }
    else
    {
        Serial.println("[Sensorboard] NEOM10S GPS Detected");
    }

    delay(150);

    // Print GPS Configuration Information

    // Serial.println("+=== GNSS SYSTEM ===+");
    // Serial.print("Module: "); Serial.println(gps->getModuleName());
    // Serial.print("Protocol Version: "); Serial.print(gps->getProtocolVersionHigh());
    // Serial.print("."); Serial.println(gps->getProtocolVersionLow());

    // Initialize starting state
    state->initialize();

    timer.reset();
}

void loop()
{
    if (timer.check() == 1)
    {
        // Reads sensors, logs to flash chip, loops the state
        state->loop();

        State *nextState = state->nextState();
        if (nextState != nullptr)
        {
            delete state;
            state = nextState;
            state->initialize();
        }
    }
}
