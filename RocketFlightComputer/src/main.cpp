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

// Start in pre-launch
State * state = new PreLaunch();

Adafruit_BNO055 * bno; // 0x28 I2C ADDR
Adafruit_LPS25 * baro; // 0x5D I2C ADDR
Adafruit_ICM20649 * icm; // 0x68 I2C ADDR

State::TelemPacket telemBufferPacket;

void setup()
{
    Serial.begin(115200);

    while(!Serial);
    Serial.println("Beginning Flight Computer");

    bno = new Adafruit_BNO055(55, 0x28);
    baro = new Adafruit_LPS25();
    icm = new Adafruit_ICM20649();

    if(!bno->begin()) {
        Serial.println("[Sensorboard] No BNO055 Detected");
        while(1);
    };
    Serial.println("[Sensorboard] BNO055 Detected");

    if(!baro->begin_I2C()) {
        Serial.println("[Sensorboard] No LPS25 Detected");
        while(1);
    };
    Serial.println("[Sensorboard] LPS25 Detected");

    if(!icm->begin_I2C()) {
        Serial.println("[Sensorboard] No ICM20649 Detected");
        while(1);
    };
    Serial.println("[Sensorboard] ICM20649 Detected");

    // LPS25 Configuration
    baro->setDataRate(LPS25_RATE_25_HZ); // Set to 25Hz rate

    // ICM20648 Configuration
    icm->setAccelRange(ICM20649_ACCEL_RANGE_16_G); // Set Accel +- 16G
    icm->setGyroRange(ICM20649_GYRO_RANGE_2000_DPS); // Set Gyro +- 2000dps
    icm->setAccelRateDivisor(4095);
    icm->setGyroRateDivisor(255);

    Wire.begin();
    Wire.setClock(400000);

    timer.reset();

    state->initialize();

    uint8_t system, gyroCal, accelCal, magCal = 0;
    bno->getCalibration(&system, &gyroCal, &accelCal, &magCal);

    Serial.println("[BNO055] +=== CALIBRATION DATA ===+");
    Serial.print("System Cal: "); Serial.println(system);
    Serial.print("Gyro Cal: "); Serial.println(gyroCal);
    Serial.print("Accel Cal: "); Serial.println(accelCal);
    Serial.print("Mag Cal: "); Serial.println(magCal);
}

void readsensors() {
    //Create and register event handler for LPS25
    sensors_event_t lpsTempEvent;
    sensors_event_t lpsPressureEvent;
    baro->getEvent(&lpsPressureEvent, &lpsTempEvent);

    //Create and register event handler for ICM20649
    sensors_event_t icmAccelEvent;
    sensors_event_t icmGyroEvent;
    sensors_event_t icmTempEvent;
    icm->getEvent(&icmAccelEvent, &icmGyroEvent, &icmTempEvent);

    //Create and register event handler for BNO055
    sensors_event_t orientationEvent, linearAccelEvent, magnetometerEvent, bnoAccelEvent;
    bno->getEvent(&orientationEvent, Adafruit_BNO055::VECTOR_EULER);
    bno->getEvent(&linearAccelEvent, Adafruit_BNO055::VECTOR_LINEARACCEL);
    bno->getEvent(&magnetometerEvent, Adafruit_BNO055::VECTOR_MAGNETOMETER);
    bno->getEvent(&bnoAccelEvent, Adafruit_BNO055::VECTOR_ACCELEROMETER);

    // Update state machine
    telemBufferPacket.accelX = bnoAccelEvent.acceleration.x; // m/s/s
    telemBufferPacket.accelY = bnoAccelEvent.acceleration.y; // m/s/s
    telemBufferPacket.accelZ = bnoAccelEvent.acceleration.z; // m/s/s

    telemBufferPacket.gyroX = icmGyroEvent.gyro.x; // 
    telemBufferPacket.gyroY = icmGyroEvent.gyro.y;
    telemBufferPacket.gyroZ = icmGyroEvent.gyro.z;

    telemBufferPacket.magX = magnetometerEvent.magnetic.x;
    telemBufferPacket.magY = magnetometerEvent.magnetic.y;
    telemBufferPacket.magZ = magnetometerEvent.magnetic.z;
    telemBufferPacket.heading = magnetometerEvent.magnetic.heading;

    telemBufferPacket.pressure = lpsPressureEvent.pressure;
    telemBufferPacket.altitude = Utility::pressureToAltitude(telemBufferPacket.pressure);
}

void loop()
{
    if (timer.check() == 1)
    {

        readsensors();
        memcpy(&state->telemPacket, &telemBufferPacket, sizeof(telemBufferPacket));
    

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