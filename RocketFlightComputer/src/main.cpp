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
        // readsensors();
        // memcpy(&state->sensorPacket, &sensorPacket, sizeof(sensorPacket));
        state->loop();

        Serial.print("Dt: "); Serial.println(millis() - previousTime);
        previousTime = millis();

        // Serial.println("<--- Accel --->");
        // Serial.print("Accel X: "); Serial.println(sensorPacket.accelX);
        // Serial.print("Accel Y: "); Serial.println(sensorPacket.accelY);
        // Serial.print("Accel Z: "); Serial.println(sensorPacket.accelZ);

        // Serial.println("<--- Gyro --->");
        // Serial.print("Gyro X: "); Serial.println(sensorPacket.gyroX);
        // Serial.print("Gyro Y: "); Serial.println(sensorPacket.gyroY);
        // Serial.print("Gyro Z: "); Serial.println(sensorPacket.gyroZ);

        // Serial.print(state->sensorPacket.accelX); Serial.print(",");
        // Serial.print(state->sensorPacket.accelY); Serial.print(",");
        // Serial.print(state->sensorPacket.accelZ); Serial.print(",");
        // Serial.print(state->sensorPacket.gyroX); Serial.print(",");
        // Serial.print(state->sensorPacket.gyroY); Serial.print(",");
        // Serial.print(state->sensorPacket.gyroZ); Serial.print(",");
        // Serial.print(state->sensorPacket.magX); Serial.print(",");
        // Serial.print(state->sensorPacket.magY); Serial.print(",");
        // Serial.print(state->sensorPacket.magZ); Serial.print(",");
        // Serial.print(state->sensorPacket.altitude); Serial.print(",");
        // Serial.print(state->sensorPacket.w); Serial.print(",");
        // Serial.print(state->sensorPacket.i); Serial.print(",");
        // Serial.print(state->sensorPacket.j); Serial.print(",");
        // Serial.print(state->sensorPacket.k); Serial.print(",");
        // Serial.print(bnoQuat.w()); Serial.print(",");
        // Serial.print(bnoQuat.x()); Serial.print(",");
        // Serial.print(bnoQuat.y()); Serial.print(",");
        // Serial.print(bnoQuat.z()); Serial.print(",");
        // Serial.println(millis());

        // Serial.print("QUAT|");
        // Serial.print(state->x_state(0)); Serial.print(",");
        // Serial.print(state->x_state(1)); Serial.print(",");
        // Serial.print(state->x_state(2)); Serial.print(",");
        // Serial.println(state->x_state(3));

//     // Serial.print("ACC|");
//     // Serial.print(sensorPacket.ac_x); Serial.print(",");
//     // Serial.print(sensorPacket.ac_y); Serial.print(",");
//     // Serial.println(sensorPacket.ac_z);

        State *nextState = state->nextState();
        if (nextState != nullptr)
        {
            delete state;
            state = nextState;
            state->initialize();
        }
    }
};
