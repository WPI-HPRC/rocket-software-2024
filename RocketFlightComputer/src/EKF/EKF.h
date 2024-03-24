#pragma once

#include "Arduino.h"
#include <ArduinoEigen.h>
#include "../../src/utility.hpp"

template<int M, int N>
using Matrix = Eigen::Matrix<double, M, N>;

template<int N>
using Vector = Eigen::Vector<double, N>;

/**
 * @author @frostydev99 - Daniel Pearson
 * @brief Quaternion State Estimator performing gyroscope and magnetometer sensor fusion to predict vehicle orientation
 */
class StateEstimator {
    public:

    StateEstimator(const Vector<10>& initialOrientation, float dt);

    Vector<10> onLoop(Utility::SensorPacket sensorPacket);

    Matrix<3, 3> quat2rotm(const Vector<4>& q);
    
    private:
    constexpr static int initialLoopIters = 1000;

    // Input variance from sensor data sheet
    const float gyroVariance = 0.00489/sqrt(40); // [Rad/s]
    const float magVariance = 0.008; // [T]
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
    float inclination = 66.546 * (PI/180); // [Rad]

    Matrix<10, 10> P {
        {1,0,0,0,0,0,0,0,0,0},
        {0,1,0,0,0,0,0,0,0,0},
        {0,0,1,0,0,0,0,0,0,0},
        {0,0,0,1,0,0,0,0,0,0},
        {0,0,0,0,1,0,0,0,0,0},
        {0,0,0,0,0,1,0,0,0,0},
        {0,0,0,0,0,0,1,0,0,0},
        {0,0,0,0,0,0,0,1,0,0},
        {0,0,0,0,0,0,0,0,1,0},
        {0,0,0,0,0,0,0,0,0,1}
    }; // Process Error Covariance

    Matrix<10, 10> P_min {
        {1,0,0,0,0,0,0,0,0,0},
        {0,1,0,0,0,0,0,0,0,0},
        {0,0,1,0,0,0,0,0,0,0},
        {0,0,0,1,0,0,0,0,0,0},
        {0,0,0,0,1,0,0,0,0,0},
        {0,0,0,0,0,1,0,0,0,0},
        {0,0,0,0,0,0,1,0,0,0},
        {0,0,0,0,0,0,0,1,0,0},
        {0,0,0,0,0,0,0,0,1,0},
        {0,0,0,0,0,0,0,0,0,1}
    }; // Process Error Covariance

    const Matrix<6, 6> R {
        {accelVariance*accelVariance, 0, 0, 0, 0, 0},
        {0, accelVariance*accelVariance, 0, 0, 0, 0},
        {0, 0, accelVariance*accelVariance, 0, 0, 0},
        {0, 0, 0, magVariance*magVariance, 0, 0},
        {0, 0, 0, 0, magVariance*magVariance, 0},
        {0, 0, 0, 0, 0, magVariance*magVariance}
    }; // Sensor Noise Covariance - Accel and Mag

    const Matrix<6, 6> gyroAccelVar {
        {gyroVariance*gyroVariance, 0, 0, 0, 0, 0},
        {0, gyroVariance*gyroVariance, 0, 0, 0, 0},
        {0, 0, gyroVariance*gyroVariance, 0, 0, 0},
        {0, 0, 0, accelVariance*accelVariance, 0, 0},
        {0, 0, 0, 0, accelVariance*accelVariance, 0},
        {0, 0, 0, 0, 0, accelVariance*accelVariance}
    };

    const Matrix<10, 10> Q_Inertial {
        {0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0.7,0,0,0,0,0},
        {0,0,0,0,0,0.7,0,0,0,0},
        {0,0,0,0,0,0,0.7,0,0,0},
        {0,0,0,0,0,0,0,0.7,0,0},
        {0,0,0,0,0,0,0,0,0.7,0},
        {0,0,0,0,0,0,0,0,0,0.7}
    }; // 10 Element Identity Matrix
    
    const Matrix<10, 10> eye10 = Matrix<10, 10>::Identity();
    // const BLA::Matrix<10,10> eye10 {
    //     1,0,0,0,0,0,0,0,0,0,
    //     0,1,0,0,0,0,0,0,0,0,
    //     0,0,1,0,0,0,0,0,0,0,
    //     0,0,0,1,0,0,0,0,0,0,
    //     0,0,0,0,1,0,0,0,0,0,
    //     0,0,0,0,0,1,0,0,0,0,
    //     0,0,0,0,0,0,1,0,0,0,
    //     0,0,0,0,0,0,0,1,0,0,
    //     0,0,0,0,0,0,0,0,1,0,
    //     0,0,0,0,0,0,0,0,0,1,
    // }; // 10 Element Identity Matrix

    Vector<10> measurementFunction(Utility::SensorPacket sensorPacket);
    Matrix<10, 10> measurementJacobian(Utility::SensorPacket sensorPacket);

    Vector<6> updateFunction(Utility::SensorPacket sensorPacket);
    Matrix<6, 10> updateJacobian(Utility::SensorPacket sensorPacket);

    Matrix<10, 6> updateModelCovariance(Utility::SensorPacket sensorPacket);

    Vector<10> x;
    Vector<10> x_min;

    float g = 9.80665;

    Vector<4> quaternionMultiplication(const Vector<4>& q1, const Vector<4>& q2);
};
