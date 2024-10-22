#include <Arduino.h>

struct SensorFrame
{
    float ac_x;  // Acceleration in g
    float ac_y; // Acceleration in g
    float ac_z; // Acceleration in g
    float gy_x; // Angular velocity in degrees/s
    float gy_y; // Angular velocity in degrees/s
    float gy_z; // Angular velocity in degrees/s 
    float mag_x;
    float mag_y;
    float mag_z;
    float Pressure; // Pressure in mBar
    float Temperature; // Temperature in degrees C
    float gps_lat;
    float gps_lon;
    float gps_satellites;
    bool gps_lock;
    float gps_alt_agl;
    float gps_alt_msl;
    uint32_t gps_epoch_time;
};
