/**
 * @file KalmanFilter.h
 * @author Daniel Pearson (djpearson@wpi.edu)
 * @brief This program contains an Extended Kalman Filter for predicting vehicle orientation as a quaternion, position and velocity.  The filter fuses data from the Accelerometer, Gyroscope and Magnetometer.  For more information contact Dan (djpearson@wpi.edu), Colette (cbscott@wpi.edu), or Nikhil (nrgangaram@wpi.edu).
 * @version 1.0
 * @date 2024-1-3
 * 
 * @copyright Copyright (c) 2023 Worcester Polytechnic Institute High Power Rocketry Club 2023
 * 
 */

#pragma once

#include <Arduino.h>
#include <ArduinoEigen.h>
#include <states/State.h>

class StateEstimator {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /**
         * @brief Construct a new State Estimator object
         * 
         * @param x0 Intial state vector
         * @param dt Timestep for discrete EKF
         */
        StateEstimator(const Eigen::Vector<float, 10> &x0, float dt);

        /**
         * @brief Called every loop of the state machine
         * 
         * @param dataPacket
         * @return Eigen::Vector<float, 10> Current state
         */
        Eigen::Vector<float, 10> onLoop(State::TelemPacket dataPacket);

        /**
         * @brief Returns the rotation matrix from a quaternion
         * 
         * @param q Quaternion
         * @return Eigen::Matrix<float, 3,3> 
         */
        Eigen::Matrix<float, 3,3> quatToRot(const Eigen::Vector<float, 4> &q);

        /**
         * @brief Multiplies two quaternions by a hamiltonian product
         * 
         * @param q1 Quaternion 1
         * @param q2 Quaternion 2
         * @return Eigen::Vector<float, 4> Product quaternion
         */
        Eigen::Vector<float, 4> quatMultiply(const Eigen::Vector<float, 4> &q1, Eigen::Vector<float,4> &q2);

    private:
        // Intialize "Actual" state
        Eigen::Vector<float, 10> x = {
            1,0,0,0,0,0,0,0,0,0
        };

        // Predicted State
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

        Eigen::Vector<float, 6> updateFunction(const Eigen::Vector<float, 6>& y);

        Eigen::Matrix<float, 6,10> updateJacobian(const Eigen::Vector<float,6>& y);

        Eigen::Matrix<float, 10,6> updateModelCovariance();

        float dt = 0;

        /*Sensor Variance*/
        const float gyroVariance = 0.0175 * sqrt(LOOP_RATE); // [dps] from ICM20649 datasheet
        const float accelVariance = 0.0027948 * sqrt(LOOP_RATE); // [m/s/s] from ICM20649 datasheet
        // const float accelVariance = 0.00069*sqrt(40);
        // const float gyroVariance = 0.00489*sqrt(40);
        const float magVariance = 0.3; // [µT] from BNO055 Datasheet

        Eigen::Matrix<float, 10,10> P = Eigen::Matrix<float, 10,10>::Identity();

        Eigen::Matrix<float, 10,10> P_min = Eigen::Matrix<float, 10,10>::Identity();

        Eigen::Matrix<float, 6,6> R {
            {accelVariance*accelVariance, 0, 0, 0, 0, 0},
            {0, accelVariance*accelVariance, 0, 0, 0, 0},
            {0, 0, accelVariance*accelVariance, 0, 0, 0},
            {0, 0, 0, magVariance*magVariance, 0, 0},
            {0, 0, 0, 0, magVariance*magVariance, 0},
            {0, 0, 0, 0, 0, magVariance*magVariance},
        };

        Eigen::Matrix<float, 6,6> gyroAccelVar {
            {gyroVariance*gyroVariance, 0, 0, 0, 0, 0},
            {0, gyroVariance*gyroVariance, 0, 0, 0, 0},
            {0, 0, gyroVariance*gyroVariance, 0, 0, 0},
            {0, 0, 0, accelVariance*accelVariance, 0, 0},
            {0, 0, 0, 0, accelVariance*accelVariance, 0},
            {0, 0, 0, 0, 0, accelVariance*accelVariance}
        };

        Eigen::Matrix<float, 3,3> softIronCal {
            {1, 0, 0},
            {0, 1, 0},
            {0, 0, 1}
        };

        const Eigen::Vector<float, 3> hardIronCal {
            pow(1,9)*2.0501, pow(1,9)*2.0465, pow(1,9)*2.3325
        };

        /* NED Constants */
        constexpr static float g0 = 9.80665; // [m/s^2] Gravitational Acceleration on Earth
        const Eigen::Vector<float, 3> G_NED = {0, 0, -g0}; // [m/s^2] Gravitational Vector in NED Frame
        constexpr static float magneticInclination = 66.546 * (PI/180); // [Rad] Magnetic Inclination

        /* Vehicle Constants */
        constexpr static float m_sys = 1; // [kg] Mass of total system



};