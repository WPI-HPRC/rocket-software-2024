/**
 * @file KalmanFilter.cpp
 * @author Daniel Pearson (djpearson@wpi.edu)
 * @brief This program contains an Extended Kalman Filter for predicting vehicle orientation as a quaternion, position and velocity.  The filter fuses data from the Accelerometer, Gyroscope and Magnetometer.  For more information contact Dan (djpearson@wpi.edu), Colette (cbscott@wpi.edu), or Nikhil (nrgangaram@wpi.edu).
 * @version 1.0
 * @date 2024-1-3
 * 
 * @copyright Copyright (c) 2023 Worcester Polytechnic Institute High Power Rocketry Club 2023
 * 
 */

#include <controls/ekf/KalmanFilter.h>

StateEstimator::StateEstimator(const Eigen::Vector<float, 10>& x0, float dt) {
    this->x = x0;
    this->dt = dt;
};

Eigen::Vector<float, 10> StateEstimator::onLoop(State::TelemPacket dataPacket) {
    // Store sensor inputs for prediction step in "u" vector
    Eigen::Vector<float, 6> u = {
        dataPacket.gyroX, dataPacket.gyroY, dataPacket.gyroZ, dataPacket.accelX, dataPacket.accelY, dataPacket.accelZ
    };

    // Predict state using the sensor inputs in the "u" vector
    x_min = predictionFunction(u);

    Eigen::Matrix<float, 10, 10> A = predictionJacobian(u);

    Eigen::Matrix<float, 10, 6> W = updateModelCovariance();

    Eigen::Matrix<float, 10, 10> Q = W * gyroAccelVar * W.transpose();

    P_min = A * P * A.transpose() + Q;

    Eigen::Vector<float, 3> magVector = {
        dataPacket.magX, dataPacket.magY, dataPacket.magZ
    };

    Eigen::Vector<float, 3> accelVector = {
        dataPacket.accelX, dataPacket.accelY, dataPacket.accelZ
    };

    // magVector = softIronCal * (magVector - hardIronCal);

    magVector = magVector / magVector.norm();
    accelVector = accelVector / accelVector.norm();

    Eigen::Vector<float, 6> z = {
        accelVector(0), accelVector(1), accelVector(2), magVector(0), magVector(1), magVector(2)
    };

    // Calculate the update step from the expected readings in the NED Frame
    Eigen::Vector<float, 6> h = updateFunction(z);

    // // Calculate the update step covariance for use in the kalman gain
    Eigen::Matrix<float, 6,10> H = updateJacobian(z);

    Eigen::Vector<float, 6> v = z - h; // Innovation

    // // Calculate the Kalman Gain
    Eigen::Matrix<float, 6,6> S = H * P_min * H.transpose() + R;
    Eigen::Matrix<float, 10,6> K = P_min * H.transpose() * S.inverse();

    // Update the corrected (posteriori) state
    x = x_min + K*v;
    // Update the corrected error covariance
    P = (Eigen::Matrix<float,10,10>::Identity() - K*H) * P_min;

    // Serial.println("<----- State Vector ----->");
    // for(int i=0; i < x.rows(); i++) {
    //     for(int j = 0; j < x.cols(); j++) {
    //         Serial.print(String(x(i,j)) + "\t");
    //     }
    //     Serial.println("");
    // }
    //Normalize quaternion state

    return this->x;

};

Eigen::Matrix<float, 3,3> StateEstimator::quatToRot(const Eigen::Vector<float, 4> &q) {
    float qw = q(0);
    float qx = q(1);
    float qy = q(2);
    float qz = q(3);

    Eigen::Matrix<float, 3,3> rotm {
        {qw*qw + qx*qx - qy*qy - qz*qz, 2 * (qx*qy - qw*qz), 2 * (qx*qz + qw*qy)},
        {2 * (qx*qy + qw*qz), qw*qw - qx*qx + qy*qy - qz*qz, 2 * (qy*qz - qw*qx)},
        {2 * (qx*qz - qw*qy), 2 * (qw*qx + qy*qz), qw*qw - qx*qx - qy*qy + qz*qz}
    };

    return rotm;
};

Eigen::Vector<float, 10> StateEstimator::predictionFunction(const Eigen::Vector<float, 6>& u) {
    float p = u(0), q = u(1), r = u(2);
    
    Eigen::Vector<float, 3> accelBody = {
        u(3), u(4), u(5)
    };

    Eigen::Vector<float, 4> f_q = {
        x(0) - (dt/2)*p*x(1) - (dt/2)*q*x(2) - (dt/2)*r*x(3),
        x(1) + (dt/2)*p*x(0) - (dt/2)*q*x(3) + (dt/2)*r*x(2),
        x(2) + (dt/2)*p*x(3) + (dt/2)*q*x(0) - (dt/2)*r*x(1),
        x(3) - (dt/2)*p*x(2) + (dt/2)*q*x(1) + (dt/2)*r*x(0)
    };

    Eigen::Vector<float, 4> quat = {x(0), x(1), x(2), x(3)};
    Eigen::Matrix<float, 3,3> rotm = quatToRot(quat);

    Eigen::Vector<float, 3> bodyGrav = rotm.transpose() * G_NED;
    Eigen::Vector<float, 3> linearAccelBody = accelBody - bodyGrav;
    Eigen::Vector<float, 3> accelNED = rotm.transpose() * linearAccelBody;

    Eigen::Vector<float, 3> f_p = {0,0,0};
    Eigen::Vector<float, 3> f_v = {0,0,0};

    // Eigen::Vector<float, 3> f_p = {
    //     x(4) + x(7)*dt + 0.5f*accelNED(0) * (dt*dt),
    //     x(5) + x(8)*dt + 0.5f*accelNED(1) * (dt*dt),
    //     x(6) + x(9)*dt + 0.5f*accelNED(2) * (dt*dt)
    // };

    // Eigen::Vector<float, 3> f_v = {
    //     x(7) + accelNED(0) * dt,
    //     x(8) + accelNED(1) * dt,
    //     x(9) + accelNED(2) * dt
    // };

    f_q = f_q  / f_q.norm();

    Eigen::Vector<float, 10> f = {
        f_q(0),
        f_q(1),
        f_q(2),
        f_q(3),
        f_v(0),
        f_v(1),
        f_v(2),
        f_p(0),
        f_p(1),
        f_p(2)
    };
    
    return f;
};

Eigen::Matrix<float, 10, 10> StateEstimator::predictionJacobian(const Eigen::Vector<float, 6>& u) {
    float p = u(0), q = u(1), r = u(2);

    Eigen::Matrix<float, 10,10> A {
        {1, -0.5f*dt*p, -0.5f*dt*q, -0.5f*dt*r, 0, 0, 0, 0, 0, 0},
        {0.5f*dt*p, 1, 0.5f*dt*r, -0.5f*dt*q, 0, 0, 0, 0, 0, 0},
        {0.5f*dt*q, -0.5f*dt*r, 1, 0.5f*dt*p, 0, 0, 0, 0, 0, 0},
        {0.5f*dt*r, 0.5f*dt*q, -0.5f*dt*p, 1, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 1, 0, 0, dt, 0, 0},
        {0, 0, 0, 0, 0, 1, 0, 0, dt, 0},
        {0, 0, 0, 0, 0, 0, 1, 0, 0, dt},
        {0, 0, 0, 0, 0, 0, 0, 1, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 1, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    };

    return A;
};

Eigen::Matrix<float, 10,6> StateEstimator::updateModelCovariance() {

    Eigen::Matrix<float, 10, 6> W {
        {-0.5f*x(1), -0.5f*x(2), -0.5f*x(3), 0, 0, 0},
        {0.5f*x(0), -0.5f*x(3), 0.5f*x(2), 0, 0, 0},
        {0.5f*x(3), 0.5f*x(0), -0.5f*x(1), 0, 0, 0},
        {-0.5f*x(2), 0.5f*x(1), 0.5f*x(0), 0, 0, 0},
        {0, 0, 0, x(4), 0, 0},
        {0, 0, 0, 0, x(5), 0},
        {0, 0, 0, 0, 0, x(6)},
        {0, 0, 0, x(4), 0, 0},
        {0, 0, 0, 0, x(5), 0},
        {0, 0, 0, 0, 0, x(6)}
    };

    return W;
}

Eigen::Vector<float, 6> StateEstimator::updateFunction(const Eigen::Vector<float, 6>& y) {
    // Eigen::Vector<float, 3> r = {
    //     cos(magneticInclination), 0, sin(magneticInclination)
    // };

    Eigen::Vector<float, 3> r = {
        20.4755, -4.5403, 46.5543
    }; // Magnetic field vector for 10314

    r = r / r.norm();
    
    Eigen::Vector<float, 3> G = G_NED / G_NED.norm();

    Eigen::Vector<float, 4> q = {x_min(0), x_min(1), x_min(2), x_min(3)};
    q = q / q.norm();

    Eigen::Matrix<float, 3,3> quatRotm = quatToRot(q);

    Eigen::Vector<float, 3> a_hat = quatRotm.transpose() * G;

    Eigen::Vector<float, 3> m_hat = quatRotm.transpose() * r;

    Eigen::Vector<float, 6> h = {
        a_hat(0), a_hat(1), a_hat(2), m_hat(0), m_hat(1), m_hat(2)
    };

    return h;
};

Eigen::Matrix<float, 6,10> StateEstimator::updateJacobian(const Eigen::Vector<float,6>& y) {
    Eigen::Vector<float, 3> r = {
        cos(magneticInclination), 0, sin(magneticInclination)
    };

    r = r / r.norm();
    
    Eigen::Vector<float, 3> G = G_NED / G_NED.norm();

    Eigen::Vector<float, 4> q = {x_min(0), x_min(1), x_min(2), x_min(3)};
    q = q / q.norm();

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

    Eigen::Matrix<float, 6,10> H {
        {gx*qw+gy*qz-gz*qy, gx*qx+gy*qy+gz*qz, -gx*qy+gy*qx-gz*qw, -gx*qz+gy*qw+gz*qx, 0, 0, 0, 0, 0, 0},
        {-gx*qz+gy*qw+gz*qx, gx*qy-gy*qx+gz*qw, gx*qx+gy*qy+gz*qz, -gx*qw-gy*qz+gz*qy, 0, 0, 0, 0, 0, 0},
        {gx*qy-gy*qx+gz*qw, gx*qz-gy*qw-gz*qx, gx*qw+gy*qz-gz*qy, gx*qx+gy*qy+gz*qz, 0, 0, 0, 0, 0, 0},
        {rx*qw+ry*qz-rz*qy, rx*qy+ry*qy+rz*qz, -rx*qy+ry*qx-rz*qw, -rx*qz+ry*qw+rz*qx, 0, 0, 0, 0, 0, 0},
        {-rx*qz+ry*qw+rz*qx, rx*qy-ry*qx+rz*qw, rx*qx+ry*qy+rz*qz, -rx*qw-ry*qz+rz*qy, 0, 0, 0, 0, 0, 0},
        {rx*qy-ry*qx+rz*qw, rx*qz-ry*qw-rz*qx, rx*qw+ry*qz-rz*qy, rx*qx+ry*qy+rz*qz, 0, 0, 0, 0, 0, 0},
    };

    return H * 2.0f;

};

Eigen::Vector<float, 4> StateEstimator::quatMultiply(const Eigen::Vector<float, 4> &q1, Eigen::Vector<float,4> &q2) {
    Eigen::Vector4f result;
    result[0] = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
    result[1] = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2];
    result[2] = q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1];
    result[3] = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0];

    return result;

};