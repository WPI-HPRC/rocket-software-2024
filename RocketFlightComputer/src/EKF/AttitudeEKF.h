#pragma once

#include "Arduino.h"
#include <BasicLinearAlgebra.h>
#include "../../src/utility.hpp"

/**
 * @author @frostydev99 - Daniel Pearson
 * @brief Quaternion State Estimator performing gyroscope and magnetometer sensor fusion to predict vehicle orientation
 */
class AttitudeStateEstimator {
    public:

    AttitudeStateEstimator();

    void init(BLA::Matrix<4> initialOrientation, float dt);

    void onLoop(Utility::TelemPacket sensorPacket);

    BLA::Matrix<3,3> quat2rotm(BLA::Matrix<4> q);

    BLA::Matrix<4> x;

    bool initialized = false;
    
    private:
    constexpr static int initialLoopIters = 1000;

    // Declare sensor variance from datasheets

    const float accelXY_Var = 0.00637; // [m/s^2]
    const float accelZ_Var = 0.00686; // [m/s^2]
    const float gyroVar = 0.000489; // [rad/s]
    const float magVar = 120; // [nT]
    float dt = 1.0 / LOOP_RATE;

    BLA::Matrix<4,4> P = {
        1,0,0,0,
        0,1,0,0,
        0,0,1,0,
        0,0,0,1
    }; // Process Error Covariance

    BLA::Matrix<4,4> P_min = {
        1,0,0,0,
        0,1,0,0,
        0,0,1,0,
        0,0,0,1
    }; // Process Error Covariance

    const BLA::Matrix<6,6> R = {
        accelXY_Var*accelXY_Var, 0, 0, 0, 0, 0,
        0, accelXY_Var*accelXY_Var, 0, 0, 0, 0,
        0, 0, accelZ_Var*accelZ_Var, 0, 0, 0,
        0, 0, 0, magVar*magVar, 0, 0,
        0, 0, 0, 0, magVar*magVar, 0,
        0, 0, 0, 0, 0, magVar*magVar
    }; // Sensor Noise Covariance - Accel and Mag

    const BLA::Matrix<3,3> Sigma_gyro = {
        gyroVar*gyroVar, 0, 0,
        0, gyroVar*gyroVar, 0,
        0, 0, gyroVar*gyroVar
    };
    
    const BLA::Matrix<4,4> eye4 = {
        1,0,0,0,
        0,1,0,0,
        0,0,1,0,
        0,0,0,1,
    }; // 10 Element Identity Matrix

    BLA::Matrix<4> measurementFunction(BLA::Matrix<6> u);
    BLA::Matrix<4,4> measurementJacobian(BLA::Matrix<6> u);

    BLA::Matrix<6> updateFunction();
    BLA::Matrix<6,4> updateJacobian();

    BLA::Matrix<4,3> updateModelCovariance(Utility::TelemPacket sensorPacket);

    BLA::Matrix<4> x_min;

    constexpr static float g = Utility::g;

    BLA::Matrix<4> quaternionMultiplication(BLA::Matrix<4> q1, BLA::Matrix<4> q2);
};
