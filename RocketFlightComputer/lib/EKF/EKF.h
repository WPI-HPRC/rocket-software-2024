#pragma once

#include "Arduino.h"
#include <BasicLinearAlgebra.h>
#include "../../src/utility.hpp"
#include "../../src/FlightParams.hpp"

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
    // const float gyroPSD = 0.002385; // [(m/s^2)^2/Hz]
    // const float accelPSD = 0.00000000000154; // [(m/s^2)^2/Hz]
    // const float magVariance = 1.2; // [mG]
    // const float gyroVariance = 0.00489/sqrt(40); // [Rad/s] 
    // const float gyroVariance = 0.000048869 * sqrt(40); // [Rad/s]
    // const float magVariance = 1.2; // [uT] Magnetometer RMS Noise
    // const float accelVariance = 0.00069/sqrt(40); // [m/s/s]
    // const float accelVariance = 0.0006864655 * sqrt(40);
    const float accelXY_Var = 0.65;
    const float accelZ_Var = 0.7;
    const float gyroVar = 0.028;
    const float magVar = 0.8;

    float dt = 1.0 / LOOP_RATE;

    /* Magnetic Field Constants for Earth at WPI */
    // float inclination = 66.546;

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
        0, 0, 0, 0, 0, magVar*magVar
    }; // Sensor Noise Covariance - Accel and Mag

    const BLA::Matrix<6,6> gyroAccelVar = {
        gyroVar*gyroVar, 0, 0, 0, 0, 0,
        0, gyroVar*gyroVar, 0, 0, 0, 0,
        0, 0, gyroVar*gyroVar, 0, 0, 0,
        0, 0, 0, accelXY_Var*accelXY_Var, 0, 0,
        0, 0, 0, 0, accelXY_Var*accelXY_Var, 0,
        0, 0, 0, 0, 0, accelZ_Var*accelZ_Var
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

    BLA::Matrix<10> measurementFunction(BLA::Matrix<6> u);
    BLA::Matrix<10,10> measurementJacobian(BLA::Matrix<6> u);

    BLA::Matrix<6> updateFunction();
    BLA::Matrix<6,10> updateJacobian();

    BLA::Matrix<10,6> updateModelCovariance(Utility::SensorPacket sensorPacket);

    BLA::Matrix<10> x;
    BLA::Matrix<10> x_min;

    float g = 9.80665;

    BLA::Matrix<4> quaternionMultiplication(BLA::Matrix<4> q1, BLA::Matrix<4> q2);
};
