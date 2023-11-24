/**
 * @file EKF.cpp
 * @author Daniel Pearson (djpearson@wpi.edu)
 * @brief This program provides an Extended Kalman Filter for predicting vehicle orientation as a quaternion.  The filter successfully fuses data from the Accelerometer, Gyroscope and Magnetometer.  For more information contact Dan, Colette or Nikhil.
 * @version 1.0
 * @date 2023-11-23
 * 
 * @copyright Copyright (c) Worcester Polytechnic Institute High Power Rocketry Club 2023
 * 
 */

#include "EKF.h"

// Import Eigen Library
#include <ArduinoEigen.h>

/**
 * @brief Construct a new Quat State Estimator:: Quat State Estimator object
 * 
 * @param x0 Initial State
 * @param dt Timestep
 */
QuatStateEstimator::QuatStateEstimator(Eigen::Vector4f x0, float dt) {
    this->x = x0;
    this->x_min = x;

    this->dt = dt;
}

/**
 * @brief Run every loop of the state machine to perform the prediction and correction step of the EKF
 * 
 * @param sensorPacket Sensor frame
 * @return Quaternionf Predicted State
 */
Eigen::Vector4f QuatStateEstimator::onLoop(SensorFrame sensorPacket) {

    /* Read Data from Sensors and Convert to SI Units */
    // Convert Accel values to m/s/s
    sensorPacket.ac_x = sensorPacket.ac_x * 9.81;
    sensorPacket.ac_y = sensorPacket.ac_y * 9.81;
    sensorPacket.ac_z = sensorPacket.ac_z * 9.81;
    // Convert gyro values from deg/s to rad/s
    sensorPacket.gy_x = sensorPacket.gy_x * (PI / 180);
    sensorPacket.gy_y = sensorPacket.gy_y * (PI / 180);
    sensorPacket.gy_z = sensorPacket.gy_z * (PI / 180);

    // Convert Gauss to microTesla
    sensorPacket.mag_x = sensorPacket.mag_x * 100;
    sensorPacket.mag_y = sensorPacket.mag_y * 100;
    sensorPacket.mag_z = sensorPacket.mag_z * 100;

    Eigen::Vector3f u = {sensorPacket.gy_x, sensorPacket.gy_y, sensorPacket.gy_z};

    x_min = predictionFunction(u);

    Eigen::Matrix4f A = predictionJacobian(u);

    Eigen::Matrix<float, 4,3> W = updateModelCovariance();

    Q = W * gyroVar * W.transpose();

    P_min = A*P*A.transpose() + Q;

    Eigen::Vector3f magVector = {
        static_cast<float>(sensorPacket.mag_x), static_cast<float>(sensorPacket.mag_y), static_cast<float>(sensorPacket.mag_z)};

    magVector = softIronCal * (magVector - hardIronCal);

    Eigen::Vector3f accelVector = {sensorPacket.ac_x, sensorPacket.ac_y, sensorPacket.ac_z};

    magVector = magVector / magVector.norm();
    accelVector = accelVector / accelVector.norm();

    Eigen::Vector<float, 6> z = {accelVector(0), accelVector(1), accelVector(2), magVector(0), magVector(1), magVector(2)};

    Eigen::Vector<float, 6> h = updateFunction();

    Eigen::Matrix<float, 6,4> H = updateJacobian();

    Eigen::Vector<float, 6> v = z - h;
    Eigen::Matrix<float, 6,6> S = H * P_min * H.transpose() + R;
    Eigen::Matrix<float, 4,6> K = P_min * H.transpose() * S.inverse();

    x = x_min + K*v;

    P = (Eigen::Matrix4f::Identity() - K*H)*P_min;

    x = x / x.norm();

    return x;


};

Eigen::Vector4f QuatStateEstimator::predictionFunction(Eigen::Vector3f u) {
    float p = u(0); float q = u(1); float r = u(2);

    Eigen::Vector4f f_q;
    f_q << x(0) - (dt/2)*p*x(1) - (dt/2)*q*x(2) - (dt/2)*r*x(3),
           x(1) + (dt/2)*p*x(0) - (dt/2)*q*x(3) + (dt/2)*r*x(2),
           x(2) + (dt/2)*p*x(3) + (dt/2)*q*x(0) - (dt/2)*r*x(1),
           x(3) - (dt/2)*p*x(2) + (dt/2)*q*x(1) + (dt/2)*r*x(0);

    f_q /= f_q.norm();

    return f_q;
};

Eigen::Matrix4f QuatStateEstimator::predictionJacobian(Eigen::Vector3f u) {
    float p = u(0); float q = u(1); float r = u(2);

    Eigen::Matrix4f A;
    A << 1, -(dt/2)*p, -(dt/2)*q, -(dt/2)*r,
         (dt/2)*p, 1, (dt/2)*r, -(dt/2)*q,
         (dt/2)*q, -(dt/2)*r, 1, (dt/2)*p,
         (dt/2)*r, (dt/2)*q, -(dt/2)*p, 1;

    return A;
};

Eigen::Matrix<float, 4,3> QuatStateEstimator::updateModelCovariance() {

    Eigen::Matrix<float, 4,3> W {
        {-x(1), -x(2), -x(3)},
        {x(0), -x(3), x(2)},
        {x(3), x(0), -x(1)},
        {-x(2), x(1), x(0)}
    };

    return (W * (dt/2.0f));
};

Eigen::Matrix<float, 6,1> QuatStateEstimator::updateFunction() {
    Eigen::Vector3f r = {20468.9, -4540.5, 46559.4};
    r /= r.norm();

    G /= G.norm();

    Eigen::Quaternionf q = {x_min(0), x_min(1), x_min(2), x_min(3)};

    Eigen::Matrix3f quatRotm = q.toRotationMatrix();

    Eigen::Vector3f a_hat = quatRotm.transpose() * G;
    Eigen::Vector3f m_hat = quatRotm.transpose() * r;

    Eigen::Vector<float, 6> h = {a_hat(0), a_hat(1), a_hat(2), m_hat(0), m_hat(1), m_hat(2)};

    return h;

};

Eigen::Matrix<float, 6,4> QuatStateEstimator::updateJacobian() {
    Eigen::Vector3f r = {20468.9, -4540.5, 46559.4};
    r /= r.norm();

    G /= G.norm();

    Eigen::Vector4f q = {x_min(0), x_min(1), x_min(2), x_min(3)};

    float rx = r(0);
    float ry = r(1);
    float rz = r(2);
    float gx = G(0);
    float gy = G(1);
    float gz = G(2);

    float qw = q(0);
    float qx = q(1);
    float qy = q(2);
    float qz = q(3);

    Eigen::Matrix<float, 6, 4> H;

    H << gx*qw + gy*qz - gz*qy, gx*qx + gy*qy + gz*qz, -gx*qy + gy*qx - gz*qw, -gx*qz + gy*qw + gz*qx,
         -gx*qz + gy*qw + gz*qx, gx*qy - gy*qx + gz*qw, gx*qx + gy*qy + gz*qz, -gx*qw - gy*qz + gz*qy,
         gx*qy - gy*qx + gz*qw, gx*qz - gy*qw - gz*qx, gx*qw + gy*qz - gz*qy, gx*qx + gy*qy + gz*qz,
         rx*qw + ry*qz - rz*qy, rx*qy + ry*qy + rz*qz, -rx*qy + ry*qx - rz*qw, -rx*qz + ry*qw + rz*qx,
         -rx*qz + ry*qw + rz*qx, rx*qy - ry*qx + rz*qw, rx*qx + ry*qy + rz*qz, -rx*qw - ry*qz + rz*qy,
         rx*qy - ry*qx + rz*qw, rx*qz - ry*qw - rz*qx, rx*qw + ry*qz - rz*qy, rx*qx + ry*qy + rz*qz;

    return (H * 2.0f);
};