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
#include <SparkFun_MMC5983MA_Arduino_Library.h>
#include <ICM42688.h>

#include <states/State.h>
#include <states/PreLaunch.h>

#include <Barometer.h>
#include <BNO055.h>
#include <GNSS.h>

#define LOOP_RATE 100

Metro timer = Metro(1000 / LOOP_RATE);

Utility::SensorPacket sensorPacket;
struct Sensors sensors = {
    .barometer = new Barometer(),
    .gnss = new GNSS(),
    .bno055 = new BNO055(0), // idk what the number in the constructor is for
};

State *state = new PreLaunch(&sensors);

void setup()
{
    Serial.begin(115200);

    while(!Serial);
    Serial.println("Beginning Flight Computer");

    Wire.begin();
    Wire.setClock(400000);


    // if(!bno->begin()) {
    //     Serial.println("[Sensorboard] No BNO055 Detected");
    // } else {
    //     Serial.println("[Sensorboard] BNO055 IMU Detected");
    // }

    if(!baro->begin_I2C(0x5C)) {
        Serial.println("[Sensorboard] No LPS25 Detected");
    } else {
        Serial.println("[Sensorboard] LPS25 Barometer Detected");
    }

    if(!mag->begin()) {
        Serial.println("[Sensorboard] No MMC5983 Detected");
    } else {
        Serial.println("[Sensorboard] MMC5983 Detected");
    }

    // if(icm->begin() < 0) {
    //     Serial.println("[Sensorboard] No ICM42688 Detected");
    // } else {
    //     Serial.println("[Sensorboard] ICM42688 Detected");
    // }

    if(!gps->begin()) {
        Serial.println("[Sensorboard] No NEOM10S Detected");
    } else {
        Serial.println("[Sensorboard] NEOM10S GPS Detected");
    }
    

    // LPS25 Configuration
    baro->setDataRate(LPS25_RATE_12_5_HZ); // Set to 12.5Hz

    mag->softReset();

    // ICM42688 Configuration
    icm->setAccelFS(ICM42688::gpm16);
    icm->setGyroFS(ICM42688::dps250);

    icm->setAccelODR(ICM42688::odr100);
    icm->setGyroODR(ICM42688::odr100);

    delay(150);

    // NEOM10S Configuration
    gps->setI2COutput(COM_TYPE_UBX);
    gps->setNavigationFrequency(20);
    gps->setAutoPVT(true);

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

    icm->getAGT();

    sensorPacket.accelX = icm->accX();
    sensorPacket.accelY = icm->accY();
    sensorPacket.accelZ = icm->accZ();

    sensorPacket.gyroX = icm->gyrX();
    sensorPacket.gyroY = icm->gyrY();
    sensorPacket.gyroZ = icm->gyrZ();

    sensorPacket.magX = mag->getMeasurementX();
    sensorPacket.magY = mag->getMeasurementY();
    sensorPacket.magZ = mag->getMeasurementZ();

    sensorPacket.pressure = lpsPressureEvent.pressure; // [hPa/mBar]
    sensorPacket.altitude = Utility::pressureToAltitude(sensorPacket.pressure); // m
    
    // Check for GPS Data Availabilty
    if(gps->getGnssFixOk()) {
        sensorPacket.gpsLock = gps->getGnssFixOk();
        sensorPacket.gpsLat = gps->getLatitude() / pow(10,7); // [deg]
        sensorPacket.gpsLong = gps->getLongitude() / pow(10,7); // [deg]
        sensorPacket.gpsAltAGL = gps->getAltitude() / 1000; // [m]
        sensorPacket.gpsAltMSL = gps->getAltitudeMSL() / 1000; // [m]
        sensorPacket.satellites = gps->getSIV();

        Serial.println(sensorPacket.gpsLat);
        Serial.println("GNSS FIX OK");

    }

    Serial.println(gps->getLatitude());
    // if(gps->getPVT()) {
    //     // GPS Lock Acquired
    //     sensorPacket.gpsLock = gps->getGnssFixOk();
    
    //     sensorPacket.gpsLat = gps->getLatitude() / pow(10,7); // [deg]
    //     sensorPacket.gpsLong = gps->getLongitude() / pow(10,7); // [deg]
    //     sensorPacket.gpsAltAGL = gps->getAltitude() / 1000; // [m]
    //     sensorPacket.gpsAltMSL = gps->getAltitudeMSL() / 1000; // [m]
    //     sensorPacket.satellites = gps->getSIV();
    //     // Serial.println("Satelltites: "); Serial.println(gps->getSIV());
        
    // } else {
    //     // Serial.print(gps->getHour()); Serial.print(":"); Serial.print(gps->getMinute()); Serial.print(":"); Serial.println(gps->getSecond());
    //     // Serial.print(gps->getMonth()); Serial.print("/"); Serial.print(gps->getDay()); Serial.print("/"); Serial.println(gps->getYear());
    //     // Serial.println(gps->getTimeOfWeek());
    //     return;
    // };
}

void loop()
{
    if (timer.check() == 1)
    {
        lastLoopTime = millis();

        readsensors();
        memcpy(&state->sensorPacket, &sensorPacket, sizeof(sensorPacket));
        state->loop();

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
