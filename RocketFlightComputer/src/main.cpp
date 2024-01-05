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

Metro timer = Metro(1000 / LOOP_RATE);

// Start in pre-launch
State * state = new PreLaunch();

Adafruit_BNO055 * bno; // 0x28 I2C ADDR
Adafruit_LPS25 * baro; // 0x5D I2C ADDR
Adafruit_ICM20649 * icm; // 0x68 I2C ADDR
SFE_UBLOX_GNSS * gps; // 0x42 I2C ADDR

State::SensorPacket telemBufferPacket;

void setup()
{
    Serial.begin(115200);

    while(!Serial);
    Serial.println("Beginning Flight Computer");

    Wire.begin();
    Wire.setClock(400000);

    bno = new Adafruit_BNO055(55, 0x28);
    baro = new Adafruit_LPS25();
    icm = new Adafruit_ICM20649();
    gps = new SFE_UBLOX_GNSS();

    if(!bno->begin()) {
        Serial.println("[Sensorboard] No BNO055 Detected");
        while(1);
    };
    Serial.println("[Sensorboard] BNO055 IMU Detected");

    if(!baro->begin_I2C()) {
        Serial.println("[Sensorboard] No LPS25 Detected");
        while(1);
    };
    Serial.println("[Sensorboard] LPS25 Barometer Detected");

    if(!icm->begin_I2C()) {
        Serial.println("[Sensorboard] No ICM20649 IMU Detected");
        while(1);
    };
    Serial.println("[Sensorboard] ICM20649 Detected");

    if(!gps->begin()) {
        Serial.println("[Sensorboard] No NEOM10S Detected");
    };
    Serial.println("[Sensorboard] NEOM10S GPS Detected");

    // LPS25 Configuration
    baro->setDataRate(LPS25_RATE_25_HZ); // Set to 25Hz rate

    // ICM20648 Configuration
    icm->setAccelRange(ICM20649_ACCEL_RANGE_16_G); // Set Accel +- 16G
    icm->setGyroRange(ICM20649_GYRO_RANGE_1000_DPS); // Set Gyro +- 2000dps

    // NEOM10S Configuration
    gps->setI2COutput(COM_TYPE_UBX);
    gps->setNavigationFrequency(40);
    gps->setAutoPVT(true);
    gps->saveConfiguration();

    //Print GPS Configuration Information

    Serial.println("+=== GNSS SYSTEM ===+");
    Serial.print("Module: "); Serial.println(gps->getModuleName());
    Serial.print("Protocol Version: "); Serial.print(gps->getProtocolVersionHigh());
    Serial.print("."); Serial.println(gps->getProtocolVersionLow());

    state->initialize();

    timer.reset();
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
    sensors_event_t orientationEvent, magnetometerEvent;
    bno->getEvent(&orientationEvent, Adafruit_BNO055::VECTOR_EULER);
    // bno->getEvent(&linearAccelEvent, Adafruit_BNO055::VECTOR_LINEARACCEL);
    bno->getEvent(&magnetometerEvent, Adafruit_BNO055::VECTOR_MAGNETOMETER);
    // bno->getEvent(&bnoAccelEvent, Adafruit_BNO055::VECTOR_ACCELEROMETER);

    // Update state machine
    telemBufferPacket.accelX = icmAccelEvent.acceleration.x; // [m/s/s]
    telemBufferPacket.accelY = icmAccelEvent.acceleration.y; // [m/s/s]
    telemBufferPacket.accelZ = icmAccelEvent.acceleration.z; // [m/s/s]

    telemBufferPacket.gyroX = icmGyroEvent.gyro.x; // [rad/s]
    telemBufferPacket.gyroY = icmGyroEvent.gyro.y; // [rad/s]
    telemBufferPacket.gyroZ = icmGyroEvent.gyro.z; // [rad/s]

    telemBufferPacket.magX = magnetometerEvent.magnetic.x; // [µT]
    telemBufferPacket.magY = magnetometerEvent.magnetic.y; // [µT]
    telemBufferPacket.magZ = magnetometerEvent.magnetic.z; // [µT]

    telemBufferPacket.pressure = lpsPressureEvent.pressure; // [hPa/mBar]
    telemBufferPacket.altitude = Utility::pressureToAltitude(telemBufferPacket.pressure); // m
    
    // Check for GPS Data Availabilty
    if(gps->getPVT()) {
        telemBufferPacket.gpsLat = gps->getLatitude() / pow(10,7); // [deg]
        telemBufferPacket.gpsLong = gps->getLongitude() / pow(10,7); // [deg]
        telemBufferPacket.gpsAltAGL = gps->getAltitude() / 1000; // [m]
        telemBufferPacket.gpsAltMSL = gps->getAltitudeMSL() / 1000; // [m]
    } else {
        return;
    };
}

float previousTime = 0;

void loop()
{
    if (timer.check() == 1)
    {
        // Serial.println(millis() - previousTime);
        previousTime = millis();
        
        readsensors();
        memcpy(&state->sensorPacket, &telemBufferPacket, sizeof(telemBufferPacket));
        state->loop();

        // Serial.print(state->sensorPacket.accelX); Serial.print(",");
        // Serial.print(state->sensorPacket.accelY); Serial.print(",");
        // Serial.print(state->sensorPacket.accelZ); Serial.print(",");
        // Serial.print(state->sensorPacket.gyroX); Serial.print(",");
        // Serial.print(state->sensorPacket.gyroY); Serial.print(",");
        // Serial.print(state->sensorPacket.gyroZ); Serial.print(",");
        // Serial.print(state->sensorPacket.magX); Serial.print(",");
        // Serial.print(state->sensorPacket.magY); Serial.print(",");
        // Serial.print(state->sensorPacket.magZ); Serial.print(",");
        // Serial.print(state->sensorPacket.q); Serial.print(",");
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

        // Serial.println(state->telemPacket.heading);
        
//     // Serial.println(x(3));
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