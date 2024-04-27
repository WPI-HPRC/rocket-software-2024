#include "EKF/EKF.h"
#include "FlightParams.hpp"
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
Servo airbrakesServo = Servo();
#ifndef NO_XBEE
XbeeProSX xbee = XbeeProSX(17); // CS GPIO17
#endif

// Metro timer = Metro(1000 / LOOP_RATE);
uint64_t lastLoopTime = 0;
uint64_t now = 0;

struct Sensors sensors;
StateEstimator *stateEstimator = new StateEstimator();
// Start in pre-launch
State *state = new PreLaunch(&sensors, stateEstimator);

// void handleMagInterrupt() {
//     sensors.mag->handleInterrupt();
// }

void setup()
{
    Serial.begin(115200);
#ifdef WAIT_FOR_SERIAL
    while (!Serial) {}
    delay(250);
#else
    delay(1000);
#endif
    Serial.println("Beginning Flight Computer");

    Wire.begin();
    Wire.setClock(400000);

    SPI.setSCK(18);
    SPI.setTX(19);
    SPI.setRX(16);
    SPI.begin();
    SPI.beginTransaction(SPISettings(6000000, MSBFIRST, SPI_MODE0));
    // SPI.beginTransaction(SPISettings(40000000, MSBFIRST, SPI_MODE0));

    // Initialize all sensors
    sensors = {
        .barometer = new Barometer(),
        .gnss = new GNSS(),
        // .bno055 = new BNO055(0),
        .mag = new Magnetometer(),
        .acc = new Accelerometer(0x68),
    };
    pinMode(SERVO_FEEDBACK_GPIO, INPUT);
    airbrakesServo.attach(SERVO_PWM_GPIO);
    airbrakesServo.write(AIRBRAKE_RETRACTED);

#ifndef NO_XBEE
    xbee.start();
#endif

    // if(!sensors.bno055->init()) {
    // #error BNO code breaks everything
    //     Serial.println("[Sensorboard] No BNO055 Detected");
    // } else {
    //     Serial.println("[Sensorboard] BNO055 IMU Detected");
    // }

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
    Wire.setClock(400000);


    // pinMode(magInterruptPin, INPUT);
    // attachInterrupt(digitalPinToInterrupt(magInterruptPin), handleMagInterrupt, RISING);

    delay(150);
    
    // Initialize starting state
    state->initialize();
}

void loop()
{
    now = millis();
    if (now - lastLoopTime >= (1000 / LOOP_RATE))
    {
        lastLoopTime = now;
        // Reads sensors, logs to flash chip, loops the state
        #ifdef PRINT_TIMINGS
        uint64_t start = millis();
        #endif
        state->loop();
        #ifdef PRINT_TIMINGS
        Serial.printf("^^ LOOP TIME: %llu\n", millis() - start);
        #endif

        State *nextState = state->nextState();
        if (nextState != nullptr)
        {
            delete state;
            state = nextState;
            state->initialize();
        }
    }
}
