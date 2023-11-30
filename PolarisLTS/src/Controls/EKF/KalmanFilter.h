/**
 * @file KalmanFilter.h
 * @author Daniel Pearson (djpearson@wpi.edu)
 * @brief This program contains an Extended Kalman Filter for predicting vehicle orientation as a quaternion, position and velocity.  The filter fuses data from the Accelerometer, Gyroscope and Magnetometer.  For more information contact Dan (djpearson@wpi.edu), Colette (cbscott@wpi.edu), or Nikhil (nrgangaram@wpi.edu).
 * @version 1.0
 * @date 2023-11-28
 * 
 * @copyright Copyright (c) 2023 Worcester Polytechnic Institute High Power Rocketry Club 2023
 * 
 */

#pragma once

#include <Arduino.h>
#include <ArduinoEigenDense.h>

#include <SensorBoardLibraries/SensorBoard.hpp>

/**
 * @brief StateEstimator
 * @author Daniel Pearson
 * 
 */
class StateEstimator {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        StateEstimator(const Eigen::Vector<float, 10>& x0, float dt);

        Eigen::Vector<float, 10> onLoop(SensorFrame sensorData);

        Eigen::Matrix<float, 3,3> quatToRotation(Eigen::Vector<float, 4> q);

    private:
        // Corrected State
        Eigen::Vector<float, 10> x = {
            1,0,0,0,0,0,0,0,0,0
        };

        // Predicted State x_tilde
        Eigen::Vector<float, 10> x_min = {
            1,0,0,0,0,0,0,0,0,0
        };

        /**
         * @brief Uses gyro and accel dynamic model of vehicle to predict current state
         * 
         * @param u vector of sensor readings for prediction
         * @return Eigen::Vector<float, 10> predicted state x_tilde
         */
        Eigen::Vector<float, 10> predictionFunction(const Eigen::Vector<float, 6>& u);

        /**
         * @brief Performs jacobian of current prediction to obtain the covariance of the change
         * 
         * @param u vector of sensor readings for prediction
         * @return Eigen::Matrix<float, 10, 10> Covariance of prediction state
         */
        Eigen::Matrix<float, 10, 10> predictionJacobian(const Eigen::Vector<float, 6>& u);

        Eigen::Vector<float, 6> updateFunction(Eigen::Vector<float, 6> y);

        Eigen::Matrix<float, 6,10> updateJacobian(Eigen::Vector<float,6> y);

        Eigen::Matrix<float, 10,6> updateModelCovariance();

        float dt = 0;

        /* Sensor Variance */
        const float gyroVariance = 0.00489/sqrt(400); // [Rad/s]
        const float magVariance = 0.008; // [T]
        const float accelVariance = 0.00069/sqrt(400); // [m/s^2]

        /* Magnetometer Calibration Matrices */
        Eigen::Matrix<float, 3,3> softIronCal {
            {3.5838, 0.8795, -1.1902},
            {0.8795, 0.7596, -0.7211},
            {-1.1902, -0.7211, 1.2511}
        };

        Eigen::Vector<float, 3> hardIronCal = {
            pow(10,7)*1.3071, pow(10,7)*1.3130, pow(10,7)*1.3042
        };

        /* EKF Matrices Initialization */
        Eigen::Matrix<float, 10,10> P = Eigen::Matrix<float, 10,10>::Identity();
        Eigen::Matrix<float, 10,10> P_min = Eigen::Matrix<float, 10,10>::Identity();

        const Eigen::Matrix<float, 6,6> R {
            {accelVariance*accelVariance, 0, 0, 0, 0, 0},
            {0, accelVariance*accelVariance, 0, 0, 0, 0},
            {0, 0, accelVariance*accelVariance, 0, 0, 0},
            {0, 0, 0, magVariance*magVariance, 0, 0},
            {0, 0, 0, 0, magVariance*magVariance, 0},
            {0, 0, 0, 0, 0, magVariance*magVariance},
        };

        const Eigen::Matrix<float, 6,6> gyroAccelVar {
            {gyroVariance*gyroVariance, 0, 0, 0, 0, 0},
            {0, gyroVariance*gyroVariance, 0, 0, 0, 0},
            {0, 0, gyroVariance*gyroVariance, 0, 0, 0},
            {0, 0, 0, gyroVariance*gyroVariance, 0, 0},
            {0, 0, 0, 0, gyroVariance*gyroVariance, 0},
            {0, 0, 0, 0, 0, gyroVariance*gyroVariance}
        };        

        /* NED Constants*/
        constexpr static float g0 = 9.80665; // [m/s^2] Gravitational Acceleration on Earth
        const Eigen::Vector<float, 3> G_NED = {0, 0, -g0}; // [m/s^2] Gravitational Vector in NED Frame
        constexpr static float magneticInclination = 66.546 * (PI/180); // [Rad] Magnetic Inclination

};