#pragma once

#include <BasicLinearAlgebra.h>

#include "FlightParams.hpp"

#ifndef NO_SDCARD
#include "SdFat.h"
#endif

#ifndef NO_SERVO
#include "Servo.h"
#endif

#ifndef NO_XBEE
#include "TelemetryBoard/XBeeProSX.h"
#endif

#include <Arduino.h>
#include <cstdint>
#include <cmath>
#include "buzzerPitches.h"

// #define DEBUG_MODE 
#define LOOP_RATE 40

#define ARRAY_SIZE(x) (sizeof(x) / sizeof(x[0]))

#define SERVO_FEEDBACK_GPIO 27
#define SERVO_PWM_GPIO 20
#define BUZZER_PIN 14

// FIXME: This seems bad but I need somewhere to track this and I don't want to have to pass it to every state constructor
#ifndef NO_SDCARD
extern bool sdCardInitialized;
extern SdFat sd;
extern File32 dataFile;
extern uint sd_spi_dma_chan;
#endif

#ifndef NO_SERVO
extern Servo airbrakesServo;
#endif

#ifndef NO_XBEE
extern XbeeProSX xbee;
#endif

class Utility
{
public:
    static float pressureToAltitude(float pressure) {
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

    static BLA::Matrix<3,3> quat2rotm(BLA::Matrix<4> q) {
        float qw = q(0);
        float qx = q(1);
        float qy = q(2);
        float qz = q(3);

        BLA::Matrix<3,3> rotm = {
                2*(qw*qw + qx*qx) - 1, 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy),
                2*(qx*qy + qw*qz), 2*(qw*qw + qy*qy) + 1, 2*(qy*qz - qw*qx),
                2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 2*(qw*qw + qz*qz) - 1,
        };

//        BLA::Matrix<3, 3> rotm = {
//            qw*qw + qx*qx - qy*qy - qz*qz, 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy),
//            2 * (qx * qy + qw * qz), qw*qw - qx*qx + qy*qy - qz*qz, 2 * (qy * qz - qw * qx),
//            2 * (qx * qz - qw * qy), 2 * (qw * qx + qy * qz), qw*qw - qx*qx - qy*qy + qz*qz
//        };

        return rotm;
    };

    float densityAtAltitude(float altitude) {

        return rho_sl * pow(temperatureAtAltitude(altitude) / T_sl, -1 - (g / (a_1 / R)));
    };

    float temperatureAtAltitude(float altitude) {

        return T_sl + a_1*altitude;
    };

    static BLA::Matrix<3> crossProduct(const BLA::Matrix<3>& vec1, const BLA::Matrix<3>& vec2) {
        BLA::Matrix<3> result;
        result(0) = vec1(1) * vec2(2) - vec1(2) * vec2(1);
        result(1) = vec1(2) * vec2(0) - vec1(0) * vec2(2);
        result(2) = vec1(0) * vec2(1) - vec1(1) * vec2(0);
        return result;
    }

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
        float temperature = 0.0f;

        // GPS Inputs
        float gpsLat = 0.0;
        float gpsLong = 0.0;
        float gpsAltMSL = 0.0;
        float gpsAltAGL = 0.0;
        int32_t gpsVelocityN = 0;
        int32_t gpsVelocityE = 0;
        int32_t gpsVelocityD = 0;
        uint32_t epochTime = 0;
        uint8_t satellites = 0;
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
        float rawMagX = 0.0f;
        float rawMagY = 0.0f;
        float rawMagZ = 0.0f;
        float pressure = 0.0f;
        float temperature = 0.0f;

        uint32_t servoPosition = 0;

        // Calculated Values
        float altitude = 0.0f;
        float magX = 0.0f;
        float magY = 0.0f;
        float magZ = 0.0f;

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
        int32_t gpsVelocityN = 0;
        int32_t gpsVelocityE = 0;
        int32_t gpsVelocityD = 0;
        uint32_t epochTime = 0;
        uint8_t satellites = 0;
        bool gpsLock = false;

        uint32_t loopCount = 0;
        uint32_t timestamp = 0;

#ifdef SERIAL_TELEMETRY
        void debugPrint() {
            Serial.print("state: "); Serial.print(state); Serial.print(", ");

            Serial.print("accelX: "); Serial.print(accelX); Serial.print(", ");
            Serial.print("accelY: "); Serial.print(accelY); Serial.print(", ");
            Serial.print("accelZ: "); Serial.print(accelZ); Serial.print(", ");
            Serial.print("gyroX: "); Serial.print(gyroX); Serial.print(", ");
            Serial.print("gyroY: "); Serial.print(gyroY); Serial.print(", ");
            Serial.print("gyroZ: "); Serial.print(gyroZ); Serial.print(", ");
            Serial.print("rawMagX: "); Serial.print(rawMagX); Serial.print(", ");
            Serial.print("rawMagY: "); Serial.print(rawMagY); Serial.print(", ");
            Serial.print("rawMagZ: "); Serial.print(rawMagZ); Serial.print(", ");

            Serial.print("pressure: "); Serial.print(pressure); Serial.print(", ");

            Serial.print("servoPosition: "); Serial.print(servoPosition); Serial.print(", ");

            Serial.print("altitude: "); Serial.print(altitude); Serial.print(", ");
            Serial.print("magX: "); Serial.print(magX); Serial.print(", ");
            Serial.print("magY: "); Serial.print(magY); Serial.print(", ");
            Serial.print("magZ: "); Serial.print(magZ); Serial.print(", ");

            Serial.print("w: "); Serial.print(w); Serial.print(", ");
            Serial.print("i: "); Serial.print(i); Serial.print(", ");
            Serial.print("j: "); Serial.print(j); Serial.print(", ");
            Serial.print("k: "); Serial.print(k); Serial.print(", ");
            Serial.print("posX: "); Serial.print(posX); Serial.print(", ");
            Serial.print("posY: "); Serial.print(posY); Serial.print(", ");
            Serial.print("posZ: "); Serial.print(posZ); Serial.print(", ");
            Serial.print("velX: "); Serial.print(velX); Serial.print(", ");
            Serial.print("velY: "); Serial.print(velY); Serial.print(", ");
            Serial.print("velZ: "); Serial.print(velZ); Serial.print(", ");

            Serial.print("gpsLat: "); Serial.print(gpsLat); Serial.print(", ");
            Serial.print("gpsLong: "); Serial.print(gpsLong); Serial.print(", ");
            Serial.print("gpsAltAGL: "); Serial.print(gpsAltAGL); Serial.print(", ");
            Serial.print("gpsAltMSL: "); Serial.print(gpsAltMSL); Serial.print(", ");
            Serial.print("gpsVelN: "); Serial.print(gpsVelocityN); Serial.print(", ");
            Serial.print("gpsVelE: "); Serial.print(gpsVelocityE); Serial.print(", ");
            Serial.print("gpsVelD: "); Serial.print(gpsVelocityD); Serial.print(", ");
            Serial.print("epochTime: "); Serial.print(epochTime); Serial.print(", ");
            Serial.print("satellites: "); Serial.print(satellites); Serial.print(", ");
            Serial.print("gpsLock: "); Serial.print(gpsLock); Serial.print(", ");

            Serial.print("loopCount: "); Serial.print(loopCount); Serial.print(", ");
            Serial.print("timestamp: "); Serial.println(timestamp);
        }
#endif
    };
    #pragma pack(pop)

    /** -----EARTH CONSTANTS----- **/
    // WGS84 Ellipsoid Model Constants
    constexpr static float a_earth = 6378137.0;       // [m] Semi-major axis of Earth
    constexpr static float b_earth = 6356752.3142;    // [m] Semi-Minor axis of Earth
    constexpr static float e_earth = 0.0818191908426; // Eccentricity of Earth
    constexpr static float r_earth = 6378137; // [m] Radius of Earth
    constexpr static float g = 9.80665; // [m/s/s] Grav Acceleration of Earth

    // Standard Atmospheric Model Constants
    constexpr static float rho_sl = 1.225; // [kg/m^3] Density at Sea Level
    constexpr static float P_sl = 101325; // [Pa] Pressure at Sea Level
    constexpr static float T_sl = 288.16; // [K] Temperature at Sea Level
    constexpr static float a_1 = -6.5e-3; // [K/m] Temperature Gradient
    constexpr static float R = 287; // [J/kgK] Universal Gas Constant
};
