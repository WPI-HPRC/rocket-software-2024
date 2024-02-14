#pragma once

#define DEBUG_MODE true
#define LOOP_RATE 40


class Utility {

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

    struct SensorPacket {
        // Raw Sensor Readings
        float accelX;
        float accelY;
        float accelZ;
        float gyroX;
        float gyroY;
        float gyroZ;
        float magX;
        float magY;
        float magZ;
        float pressure;

        // Calculated Values
        float altitude;

        // GPS Inputs
        float gpsLat;
        float gpsLong;
        float gpsAltMSL;
        float gpsAltAGL;
        uint32_t epochTime;
        uint8_t satellites;
        boolean gpsLock = false;

        uint32_t timestamp;
    };

    struct TelemPacket {
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
        boolean gpsLock = false;

        uint32_t timestamp = 0;
    };

    // WGS84 Ellipsoid Model
    constexpr static float a_earth = 6378137.0; // [m] Semi-major axis of Earth
    constexpr static float b_earth = 6356752.3142; // [m] Semi-Minor axis of Earth
    constexpr static float e_earth = 0.0818191908426; // Eccentricity of Earth
    constexpr static float r_earth = 6378137; // [m] Radius of Earth
};