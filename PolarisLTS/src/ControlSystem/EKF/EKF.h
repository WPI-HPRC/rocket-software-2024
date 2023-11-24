/**
 * @file EKF.h
 * @author Daniel Pearson (djpearson@wpi.edu)
 * @brief This program provides an Extended Kalman Filter for predicting vehicle orientation as a quaternion.  The filter successfully fuses data from the Accelerometer, Gyroscope and Magnetometer.  For more information contact Dan, Colette or Nikhil.
 * @version 1.0
 * @date 2023-11-23
 * 
 * @copyright Copyright (c) Worcester Polytechnic Institute High Power Rocketry Club 2023
 * 
 */

#pragma once

#include <Arduino.h>
#include <ArduinoEigen.h>

// Import Sensor Board Libraries
#include <SensorBoardLibraries/SensorBoard.hpp>
/**
 * @class QuatStateEstimator
 * @author Daniel Pearson (djpearson@wpi.edu)
 * @brief Quaternion state estimator implementation
 */
class QuatStateEstimator {

public:
    /**
     * @brief Construct a new Quat State Estimator:: Quat State Estimator object
     * 
     * @param x0 Initial State
     * @param dt Timestep
     */
    QuatStateEstimator(Eigen::Vector4f x0, float dt);

    /**
     * @brief Called every loop to update vehicle state
     * 
     * @param sensorPacket Sensorboard frame to pass new data to EKF
     * @return Vector4f Corrected State after sensor fusion
     */
    Eigen::Vector4f onLoop(SensorFrame sensorPacket);

private:

    /**
     * @brief Uses gyroscope dynamic model of vehicle to predict current state
     * 
     * @param u Gyroscope data vector
     * @return Vector4f Integrated gyroscope predicted orientation
     */
    Eigen::Vector4f predictionFunction(Eigen::Vector3f u);
    
    /**
     * @brief Performs jacobian of current prediction to obtain the covariance of the change
     * 
     * @param u Gyroscope data vector 
     * @return Matrix4f Covariance of gyroscope dynamic model
     */
    Eigen::Matrix4f predictionJacobian(Eigen::Vector3f u);

    /**
     * @brief Correction function applied to predicted orientation to "correct" or "verify" integrated readings weighting the validity of the sensors in the process
     * 
     * @return Matrix<float, 6,1> Unit vector of expected orientation in NED (North-East-Down) model
     */
    Eigen::Matrix<float, 6,1> updateFunction();

    /**
     * @brief Correction function covariance for weighting readings
     * 
     * @return Matrix<float, 6,4> Covariance of correction model
     */
    Eigen::Matrix<float, 6,4> updateJacobian();

    /**
     * @brief Function to return the predicted error of the dynamic model for updating the Dynamic Model Error - Q
     * 
     * @return Matrix<float, 4,3> Quaternion Error Covariance
     */
    Eigen::Matrix<float, 4,3> updateModelCovariance();
    
    // Loop rate in s
    float dt = 0.025; // Default value -> 40Hz

    const float gyroVariance = 0.00489/sqrt(dt); // [Rad/s]
    const float magVariance = 0.008; // [nT]
    const float accelVariance = 0.00069/sqrt(dt); // [m/s/s]

    /* MAGNETOMETER CALIBRATION VALUES */
    const Eigen::Matrix3f softIronCal {
        {3.5838, 0.8795, -1.1902},
        {0.8795, 0.7596, -0.7211},
        {-1.1902, -0.7211, 1.2511}
    };

    const Eigen::Vector3f hardIronCal = {
        pow(10, 7) * 1.3071, pow(10, 7) * 1.3130, pow(10, 7) * 1.3042
    };

    /* Extended Kalman Filter Initial Configuration*/

    // Process Error Covariance Priori and Posterior
    Eigen::Matrix4f P = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f P_min = Eigen::Matrix4f::Identity();

    // Sensor Noise Covariance
    Eigen::Matrix<float, 6,6> R = Eigen::Matrix<float,6,6>::Identity() * pow(accelVariance,2);

    // Gyro Noise Covariance
    Eigen::Matrix3f gyroVar = Eigen::Matrix3f::Identity() * pow(gyroVariance,2);

    // Initial Dynamic Model Covariance
    Eigen::Matrix4f Q = Eigen::Matrix4f::Identity() * 0.01;

    // State Vector
    Eigen::Vector4f x, x_min;


    /*Earth Constants*/
    constexpr static float g0 = 9.80665; // [m/s/s] Gravitational Acceleration
    constexpr static float rho_inf = 1.22; // [kg/m^3] Air Density at Sea Level

    /*NED Constants*/
    Eigen::Vector3f G = {0,0,-g0};

};