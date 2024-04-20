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

#define PIN_SPI_MISO 16
#define PIN_SPI_MOSI 19
#define PIN_SPI_SCK 18

bool sdCardInitialized = false;
File dataFile;
SdFat sd;
// Servo airbrakesServo = Servo();
XbeeProSX xbee = XbeeProSX(17); // CS GPIO17

Metro timer = Metro(1000 / LOOP_RATE);

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
#endif
    Serial.println("Beginning Flight Computer");

    Wire.begin();
    Wire.setClock(400000);

    // SPI.setSCK(18);
    // SPI.setTX(19);
    // SPI.setRX(16);
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
    pinMode(SERVO_PWM_GPIO, OUTPUT);
    // airbrakesServo.attach(SERVO_PWM_GPIO);
    // airbrakesServo.write(AIRBRAKE_RETRACTED);

    xbee.start();

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

    // pinMode(magInterruptPin, INPUT);
    // attachInterrupt(digitalPinToInterrupt(magInterruptPin), handleMagInterrupt, RISING);

    delay(150);

    // Serial.println("[Init] Try launch core 1");
    // multicore_launch_core1(handleServoPwm);
    // Serial.println("[Init] Launched core 1");
    
    // Initialize starting state
    // Serial.println("[Init] Initialize first state");
    state->initialize();
    // Serial.println("[Init] Success");

    // Serial.println("[Init] Reset timer");
    timer.reset();
    // Serial.println("[Init] Success");
}

void loop()
{
    if (timer.check() == 1)
    {
        // Reads sensors, logs to flash chip, loops the state
        // Serial.println("Start loop");
        state->loop();
        // Serial.println("Loop finished");

        State *nextState = state->nextState();
        if (nextState != nullptr)
        {
            delete state;
            state = nextState;
            state->initialize();
        }
    }
}

// void handleServoPwm() {
//     constexpr uint64_t microseconds = 1000000 / 50;
//     uint64_t lastTime = micros();
//     int on = false;
//     uint64_t time = 0;
//     while (true) {
//         Serial.println("[Core1] Loop start");
//         if (multicore_fifo_rvalid()) {
//             Serial.println("[Core1] Read from fifo");
//             time = multicore_fifo_pop_blocking();
//             Serial.println("[Core1] Success");
//         }
//         uint64_t now = micros();
//         if (on && now - lastTime >= time) {
//             on = false;
//         } else if (!on && now - lastTime >= microseconds - time) {
//             on = true;
//         }
//         lastTime = now;
//         digitalWrite(SERVO_PWM_GPIO, on);
//         delayMicroseconds(50);
//     }
// }
