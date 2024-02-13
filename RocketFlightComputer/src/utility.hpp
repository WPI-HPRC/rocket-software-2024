#pragma once
#include <Flash.h>

#define DEBUG_MODE true
#define LOOP_RATE 40
#define G 9.81

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
        // State Integer
        // 0 - PreLaunch
        // 1 - Launch
        // 2 - Coast
        // 3 - DrogueDescent
        // 4 - MainDescent
        // 5 - Recovery
        // 6 - Abort
        uint8_t state;

        // Raw Sensor Readings
        float accelX;
        float accelY;
        float accelZ;
        float gyroX;
        float gyroY;
        float gyroZ;
        uint32_t magX;
        uint32_t magY;
        uint32_t magZ;
        float pressure;

        // Calculated Values
        float altitude;

        // State Estimator Outputs
        float w;
        float i;
        float j;
        float k;

        // Geocentric Position
        float X;
        float Y;
        float Z;

        // GPS Inputs
        float gpsLat;
        float gpsLong;
        float gpsAltMSL;
        float gpsAltAGL;
        uint8_t satellites;
        boolean gpsLock = false;

        long timestamp;
    };

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

    // WGS84 Ellipsoid Model
    constexpr static float a_earth = 6378137.0;       // [m] Semi-major axis of Earth
    constexpr static float b_earth = 6356752.3142;    // [m] Semi-Minor axis of Earth
    constexpr static float e_earth = 0.0818191908426; // Eccentricity of Earth
};
