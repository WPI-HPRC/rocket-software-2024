#pragma once

#include "FS.h"
#include "Servo.h"
#include <Arduino.h>
#include <cstdint>
#include <cmath>

// #define DEBUG_MODE 
#define LOOP_RATE 40

#define ARRAY_SIZE(x) (sizeof(x) / sizeof(x[0]))

#define SERVO_FEEDBACK_GPIO 26
#define SERVO_PWM_GPIO 25

// FIXME: This seems bad but I need somewhere to track this and I don't want to have to pass it to every state constructor
extern bool sdCardInitialized;
extern fs::File dataFile;
extern Servo airbrakesServo;

class Utility
{
public:
    static float pressureToAltitude(float pressure)
    {
        // physical parameters for model
        const float pb = 101325;   // [Pa] pressure at sea level
        const float Tb = 288.15;   // [K] temperature at seal level
        const float Lb = -0.0065;  // [K/m] standard temperature lapse rate
        const float hb = 0;        // [m] height at bottom of atmospheric layer (sea level)
        const float R = 8.31432;   // [N*m/mol*K] universal gas constant
        const float g0 = 9.80665;  // [m/s^2] Earth standard gravity
        const float M = 0.0289644; // [kg/mol] molar mass of Earth's air

        // convert pressure from [hPa] to [Pa]
        float pressure_Pa = pressure * 100;

        // compute altitude from formula
        return hb + (Tb / Lb) * (pow((pressure_Pa / pb), (-R * Lb / (g0 * M))) - 1);
    };

    struct SensorPacket
    {
        // Raw Sensor Readings
        float accelX = 0.0;
        float accelY = 0.0;
        float accelZ = 0.0;
        float gyroX = 0.0;
        float gyroY = 0.0;
        float gyroZ = 0.0;
        float magX = 0.0;
        float magY = 0.0;
        float magZ = 0.0;
        float pressure = 0.0;

        // GPS Inputs
        float gpsLat;
        float gpsLong;
        float gpsAltMSL;
        float gpsAltAGL;
        uint32_t epochTime;
        uint8_t satellites;
        bool gpsLock = false;

        uint32_t timestamp = 0;
    };

    #pragma pack(push,1)
    struct TelemPacket {
        uint8_t packetType = 0x01;
        // State Integer
        // 0 - PreLaunch
        // 1 - Launch
        // 2 - Coast
        // 3 - DrogueDescent
        // 4 - MainDescent
        // 5 - Recovery
        // 6 - Abort
        uint8_t state = 0;
        // Raw Sensor Readings
        float accelX = 0.0f;
        float accelY = 0.0f;
        float accelZ = 0.0f;
        float gyroX = 0.0f;
        float gyroY = 0.0f;
        float gyroZ = 0.0f;
        float magX = 0.0f;
        float magY = 0.0f;
        float magZ = 0.0f;
        float pressure = 0.0f;

        uint32_t servoPosition = 0;

        // Calculated Values
        float altitude = 0.0f;

        // EKF Results
        float w = 0.0f; // Quaternion State
        float i = 0.0f;
        float j = 0.0f;
        float k = 0.0f;
        float posX = 0.0f; // Position State ECEF
        float posY = 0.0f;
        float posZ = 0.0f;
        float velX = 0.0f; // Velocity State ECEF
        float velY = 0.0f;
        float velZ = 0.0f;

        // GPS Inputs
        float gpsLat = 0.0f;
        float gpsLong = 0.0f;
        float gpsAltMSL = 0.0f;
        float gpsAltAGL = 0.0f;
        uint32_t epochTime = 0;
        uint8_t satellites = 0;
        bool gpsLock = false;

        uint32_t loopCount = 0;
        uint32_t timestamp = 0;

#ifdef SERIAL_TELEMETRY
        void debugPrint() {
            Serial.println("state: " + String(state) + ", "

                        + "accelX: " + String(accelX) + ", "
                        + "accelY: " + String(accelY) + ", "
                        + "accelZ: " + String(accelZ) + ", "
                        + "gyroX: " + String(gyroX) + ", "
                        + "gyroY: " + String(gyroY) + ", "
                        + "gyroZ: " + String(gyroZ) + ", "
                        + "magX: " + String(magX) + ", "
                        + "magY: " + String(magY) + ", "
                        + "magZ: " + String(magZ) + ", "

                        + "pressure: " + String(pressure) + ", "
                        + "altitude: " + String(altitude) + ", "

                        + "w: " + String(w) + ", "
                        + "i: " + String(i) + ", "
                        + "j: " + String(j) + ", "
                        + "k: " + String(k) + ", "
                        + "posX: " + String(posX) + ", "
                        + "posY: " + String(posY) + ", "
                        + "posZ: " + String(posZ) + ", "
                        + "velX: " + String(velX) + ", "
                        + "velY: " + String(velY) + ", "
                        + "velZ: " + String(velZ) + ", "

                        + "gpsLat: " + String(gpsLat) + ", "
                        + "gpsLong: " + String(gpsLong) + ", "
                        + "gpsAltAGL: " + String(gpsAltAGL) + ", "
                        + "gpsAltMSL: " + String(gpsAltMSL) + ", "
                        + "epochTime: " + String(epochTime) + ", "
                        + "satellites: " + String(satellites) + ", "
                        + "gpsLock: " + String(gpsLock) + ", "

                        + "loopCount: " + String(loopCount) + ", "
                        + "timestamp: " + String(timestamp));
        }
#endif
    };
    #pragma pack(pop)

#if 0 // NOTE: REMOVES FLASH CODE
    static void logData(FlashChip *flash, SensorPacket sensorPacket)
    {
        String structString = String(sensorPacket.accelX) + "," +
                              String(sensorPacket.accelY) + "," +
                              String(sensorPacket.accelZ) + "," +
                              String(sensorPacket.gyroX) + "," +
                              String(sensorPacket.gyroY) + "," +
                              String(sensorPacket.gyroZ) + "," +
                              String(sensorPacket.magX) + "," +
                              String(sensorPacket.magY) + "," +
                              String(sensorPacket.magZ) + "," +
                              String(sensorPacket.pressure) + "," +
                              String(sensorPacket.altitude) + "," +
                              String(sensorPacket.w) + "," +
                              String(sensorPacket.i) + "," +
                              String(sensorPacket.j) + "," +
                              String(sensorPacket.k) + "," +
                              String(sensorPacket.X) + "," +
                              String(sensorPacket.Y) + "," +
                              String(sensorPacket.Z) + "," +
                              String(sensorPacket.gpsLat) + "," +
                              String(sensorPacket.gpsLong) + "," +
                              String(sensorPacket.gpsAltMSL) + "," +
                              String(sensorPacket.gpsAltAGL) + "," +
                              String(sensorPacket.gpsLock) + "," +
                              String(sensorPacket.satellites) + "," +
                              String(sensorPacket.timestamp);
        flash->writeStruct(structString);
    }
#endif // NOTE: REMOVES FLASH CODE

    // WGS84 Ellipsoid Model
    constexpr static float a_earth = 6378137.0;       // [m] Semi-major axis of Earth
    constexpr static float b_earth = 6356752.3142;    // [m] Semi-Minor axis of Earth
    constexpr static float e_earth = 0.0818191908426; // Eccentricity of Earth
    constexpr static float r_earth = 6378137; // [m] Radius of Earth
};
