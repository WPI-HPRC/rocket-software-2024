#pragma once

#include "Arduino.h"
#include <BasicLinearAlgebra.h>
#include "../../src/utility.hpp"

/**
 * @author @frostydev99 - Daniel Pearson
 * @brief Quaternion State Estimator performing gyroscope and magnetometer sensor fusion to predict vehicle orientation
 */
class StateEstimator {
    public:

    StateEstimator(BLA::Matrix<10> initialOrientation, float dt);

    BLA::Matrix<10> onLoop(Utility::SensorPacket sensorPacket);

    BLA::Matrix<3,3> quat2rotm(BLA::Matrix<4> q);
    
    private:
    constexpr static int initialLoopIters = 1000;

    // Input variance from sensor data sheet
    const float gyroVariance = 0.00489/sqrt(40); // [Rad/s] 
    const float magVariance = 0.04; // [uT] Magnetometer RMS Noise
    const float accelVariance = 0.00069/sqrt(40); // [m/s/s]

    float dt = 1.0 / LOOP_RATE;

    // Low Pass Filter Definitions
    const float alpha = 0.2;
    const int bufferSize = 10;

    float * accelXBuffer;
    float * accelYBuffer;
    float * accelZBuffer;
    int bufferIndex;

    /* Magnetic Field Constants for Earth at WPI */
    // float inclination = 66.546;
    float inclination = -11.90 * (PI/180); // [Rad]

    BLA::Matrix<10,10> P = {
        1,0,0,0,0,0,0,0,0,0,
        0,1,0,0,0,0,0,0,0,0,
        0,0,1,0,0,0,0,0,0,0,
        0,0,0,1,0,0,0,0,0,0,
        0,0,0,0,1,0,0,0,0,0,
        0,0,0,0,0,1,0,0,0,0,
        0,0,0,0,0,0,1,0,0,0,
        0,0,0,0,0,0,0,1,0,0,
        0,0,0,0,0,0,0,0,1,0,
        0,0,0,0,0,0,0,0,0,1,
    }; // Process Error Covariance

    BLA::Matrix<10,10> P_min = {
        1,0,0,0,0,0,0,0,0,0,
        0,1,0,0,0,0,0,0,0,0,
        0,0,1,0,0,0,0,0,0,0,
        0,0,0,1,0,0,0,0,0,0,
        0,0,0,0,1,0,0,0,0,0,
        0,0,0,0,0,1,0,0,0,0,
        0,0,0,0,0,0,1,0,0,0,
        0,0,0,0,0,0,0,1,0,0,
        0,0,0,0,0,0,0,0,1,0,
        0,0,0,0,0,0,0,0,0,1,
    }; // Process Error Covariance

    const BLA::Matrix<6,6> R = {
        accelVariance*accelVariance, 0, 0, 0, 0, 0,
        0, accelVariance*accelVariance, 0, 0, 0, 0,
        0, 0, accelVariance*accelVariance, 0, 0, 0,
        0, 0, 0, magVariance*magVariance, 0, 0,
        0, 0, 0, 0, magVariance*magVariance, 0,
        0, 0, 0, 0, 0, magVariance*magVariance,
    }; // Sensor Noise Covariance - Accel and Mag

    const BLA::Matrix<6,6> gyroAccelVar = {
        gyroVariance*gyroVariance, 0, 0, 0, 0, 0,
        0, gyroVariance*gyroVariance, 0, 0, 0, 0,
        0, 0, gyroVariance*gyroVariance, 0, 0, 0,
        0, 0, 0, accelVariance*accelVariance, 0, 0,
        0, 0, 0, 0, accelVariance*accelVariance, 0,
        0, 0, 0, 0, 0, accelVariance*accelVariance
    };

    // const BLA::Matrix<10,10> Q = {
    //     0.07,0,0,0,0,0,0,0,0,0,
    //     0,0.07,0,0,0,0,0,0,0,0,
    //     0,0,0.07,0,0,0,0,0,0,0,
    //     0,0,0,0.07,0,0,0,0,0,
    //     0,0,0,0,0,0,0,0,0,0,
    //     0,0,0,0,0,0,0,0,0,0,
    //     0,0,0,0,0,0,0,0,0,0,
    //     0,0,0,0,0,0,0,0,0,0,
    //     0,0,0,0,0,0,0,0,0,0,
    //     0,0,0,0,0,0,0,0,0,0,
    // }; // 10 Element Identity Matrix
    
    const BLA::Matrix<10,10> eye10 = {
        1,0,0,0,0,0,0,0,0,0,
        0,1,0,0,0,0,0,0,0,0,
        0,0,1,0,0,0,0,0,0,0,
        0,0,0,1,0,0,0,0,0,0,
        0,0,0,0,1,0,0,0,0,0,
        0,0,0,0,0,1,0,0,0,0,
        0,0,0,0,0,0,1,0,0,0,
        0,0,0,0,0,0,0,1,0,0,
        0,0,0,0,0,0,0,0,1,0,
        0,0,0,0,0,0,0,0,0,1,
    }; // 10 Element Identity Matrix

    BLA::Matrix<10> measurementFunction(Utility::SensorPacket sensorPacket);
    BLA::Matrix<10,10> measurementJacobian(Utility::SensorPacket sensorPacket);

    BLA::Matrix<6> updateFunction(Utility::SensorPacket sensorPacket);
    BLA::Matrix<6,10> updateJacobian(Utility::SensorPacket sensorPacket);

    BLA::Matrix<10,6> updateModelCovariance(Utility::SensorPacket sensorPacket);

    BLA::Matrix<10> x;
    BLA::Matrix<10> x_min;

    float g = 9.80665;

    BLA::Matrix<4> quaternionMultiplication(BLA::Matrix<4> q1, BLA::Matrix<4> q2);
};
