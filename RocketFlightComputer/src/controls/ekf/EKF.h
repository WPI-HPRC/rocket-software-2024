#pragma once

#include "Arduino.h"
#include <BasicLinearAlgebra.h>
#include <utility.hpp>

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
    const float gyroVariance = 0.00489/sqrt(100); // [Rad/s]
    const float magVariance = 0.008; // [T]
    const float accelVariance = 0.00069/sqrt(100); // [m/s/s]

    float dt = 0.025;

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

    /** Hard Iron Calibration */
    // **WARNING** THIS CALIBRATION MUST BE DONE WITH ANY SIGNIFICANT CHANGE IN NEARFIELD OF SENSOR
        // Refer to controlsystem_2023\StateEstimator\MagnetometerCal for further information
    BLA::Matrix<3,3> magSoftIron = {
        1.3518,    0.1398,    0.0016,
        0.1398,    0.9389,   -0.2211,
        0.0016,   -0.2211,    0.8532,
    };

    BLA::Matrix<3> hardIronCal = {
        1.3497*100000, 1.3211*100000, 1.3836*100000
    };

    BLA::Matrix<10> measurementFunction(Utility::SensorPacket sensorPacket);
    BLA::Matrix<10,10> measurementJacobian(Utility::SensorPacket sensorPacket);

    BLA::Matrix<6> updateFunction(Utility::SensorPacket sensorPacket);
    BLA::Matrix<6,10> updateJacobian(Utility::SensorPacket sensorPacket);

    BLA::Matrix<10,6> updateModelCovariance(Utility::SensorPacket sensorPacket);

    BLA::Matrix<10> x;
    BLA::Matrix<10> x_min;

    float G = 9.81;

    BLA::Matrix<4> quaternionMultiplication(BLA::Matrix<4> q1, BLA::Matrix<4> q2);
};