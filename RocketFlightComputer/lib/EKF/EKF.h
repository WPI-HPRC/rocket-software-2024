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
    // const float gyroVariance = 0.00489/sqrt(40); // [Rad/s]
    // const float magVariance = 0.008; // [T]
    // const float accelVariance = 0.00069/sqrt(40); // [m/s/s]
    const float accelXY_Var = 0.65; // [mg] - 100Hz bandwidth
    const float accelZ_Var = 0.7; // [mg] - 100Hz bandwidth
    const float gyroVar = 0.028; // [d/s] - 100Hz bandiwdth
    const float magVar = 0.8; // [mG] - 400Hz bandwidth

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
    float inclination = 66.546 * (PI/180); // [Rad]

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
        accelXY_Var*accelXY_Var, 0, 0, 0, 0, 0,
        0, accelXY_Var*accelXY_Var, 0, 0, 0, 0,
        0, 0, accelZ_Var*accelZ_Var, 0, 0, 0,
        0, 0, 0, magVar*magVar, 0, 0,
        0, 0, 0, 0, magVar*magVar, 0,
        0, 0, 0, 0, 0, magVar*magVar,
    }; // Sensor Noise Covariance - Accel and Mag

    const BLA::Matrix<6,6> gyroAccelVar = {
        gyroVar*gyroVar, 0, 0, 0, 0, 0,
        0, gyroVar*gyroVar, 0, 0, 0, 0,
        0, 0, gyroVar*gyroVar, 0, 0, 0,
        0, 0, 0, accelXY_Var*accelXY_Var, 0, 0,
        0, 0, 0, 0, accelXY_Var*accelXY_Var, 0,
        0, 0, 0, 0, 0, accelZ_Var*accelZ_Var
    };

    const BLA::Matrix<10,10> Q_Inertial = {
        0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0.7,0,0,0,0,0,
        0,0,0,0,0,0.7,0,0,0,0,
        0,0,0,0,0,0,0.7,0,0,0,
        0,0,0,0,0,0,0,0.7,0,0,
        0,0,0,0,0,0,0,0,0.7,0,
        0,0,0,0,0,0,0,0,0,0.7,
    }; // 10 Element Identity Matrix
    
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
