#include "MadgwickOrientation.h"

Madgwick::Madgwick(const Eigen::Vector<float, 4> &q_0, float gain) {
    this->q = q_0;
    this->gain = gain;
};

Eigen::Vector<float, 4> Madgwick::onLoop(Utility::SensorPacket sensorPacket) {

    /* Read Data from Sensors and Convert to SI Units */

    // Convert Accel values to m/s/s
    sensorPacket.accelX = sensorPacket.accelX * Utility::g;
    sensorPacket.accelY = sensorPacket.accelY * Utility::g;
    sensorPacket.accelZ = sensorPacket.accelZ * Utility::g;

    // Convert gyro values from deg/s to rad/s
    sensorPacket.gyroX = sensorPacket.gyroX * (PI / 180);
    sensorPacket.gyroY = sensorPacket.gyroY * (PI / 180);
    sensorPacket.gyroZ = sensorPacket.gyroZ * (PI / 180);

    Eigen::Vector<float, 4> gyrQuat {0, sensorPacket.gyroX, sensorPacket.gyroY, sensorPacket.gyroZ};

    Eigen::Vector<float, 3> acc {sensorPacket.accelX, sensorPacket.accelY, sensorPacket.accelZ};

    Eigen::Vector<float, 3> mag {sensorPacket.magX, sensorPacket.magY, sensorPacket.magZ};

    // Perform quaternion derivative with gyro update
    Eigen::Vector<float, 4> qDot = 0.5f * quatMultiply(q, gyrQuat);

    // Serial.println("<----- QDot ----->");
    // for (int i = 0; i < qDot.rows(); i++) {
    //     for (int j = 0; j < qDot.cols(); j++) {
    //         Serial.print(String(qDot(i,j)) + "\t");
    //     }
    //     Serial.println("");
    // }

    Eigen::Vector<float, 6> f = updateFunction(acc, mag);
    Eigen::Matrix<float, 6,4> J = updateJacobian(acc, mag);

    
    Eigen::Vector<float, 4> gradient = J.transpose() * f;

    // Normalize Gradient Descent
    gradient /= gradient.norm();

    qDot -= this->gain * gradient;

    this->q = q + qDot*dt;
    this->q /= q.norm();

    return this->q;
};

Eigen::Vector<float, 6> Madgwick::updateFunction(const Eigen::Vector<float, 3> &acc, const Eigen::Vector<float, 3> &mag) {

    // Normalize Magnetometer Readings

    Eigen::Vector<float, 3> magNorm = mag / mag.norm();

    Eigen::Vector<float, 4> magQuat = {0, magNorm(0), magNorm(1), magNorm(2)};

    Eigen::Vector<float, 3> accNorm = acc / acc.norm();

    // Rotate Magnetometer Readings into NED

    Eigen::Vector<float, 4> h = quatMultiply(this->q, quatMultiply(magQuat, quatConj(this->q)));

    float bx = sqrt(pow(h(1),2) + pow(h(2),2));
    float bz = h(3);

    // Extract quaternion components for readability in computation
    float qw = this->q(0);
    float qx = this->q(1);
    float qy = this->q(2);
    float qz = this->q(3);

    Eigen::Vector<float, 6> f;

    f(0) = 2.0f * (qx*qz - qw*qy) - accNorm(0);
    f(1) = 2.0f * (qw*qx + qy*qz) - accNorm(1);
    f(2) = 2.0f * (0.5-pow(qx,2) - pow(qy,2)) - accNorm(2);
    f(3) = 2.0f * bx * (0.5 - pow(qy, 2) - pow(qz, 2)) + 2.0f * bz * (qx*qz - qw*qy) - magNorm(0);
    f(4) = 2.0f * bx * (qx*qy - qw*qz)                 + 2.0f * bz * (qw*qx + qy*qz) - magNorm(1);
    f(5) = 2.0f * bx * (qw*qy + qx*qz)                 + 2.0f * bz * (0.5f - pow(qx,2) - pow(qy,2)) - magNorm(2);

    return f;
};

Eigen::Matrix<float, 6,4> Madgwick::updateJacobian(const Eigen::Vector<float, 3> &acc, const Eigen::Vector<float, 3> &mag) {
    // Extract quaternion components for readability in computation
    float qw = this->q(0);
    float qx = this->q(1);
    float qy = this->q(2);
    float qz = this->q(3);

    // Normalize Magnetometer Readings

    Eigen::Vector<float, 3> magNorm = mag / mag.norm();

    Eigen::Vector<float, 4> magQuat = {0, magNorm(0), magNorm(1), magNorm(2)};

    // Rotate Magnetometer Readings into NED

    Eigen::Vector<float, 4> h = quatMultiply(this->q, quatMultiply(magQuat, quatConj(this->q)));

    float bx = sqrt(pow(h(1),2) + pow(h(2),2));
    float bz = h(3);

    Eigen::Matrix<float, 6,4> J {
        {-2.0f*qy, 2.0f*qz, -2.0f*qw, 2.0f*qx},
        { 2.0f*qx,  2.0f*qw,  2.0f*qz, 2.0f*qy},
        { 0.0f,    -4.0f*qx, -4.0f*qy, 0.0f   },
        {-2.0f*bz*qy, 2.0f*bz*qz, -4.0f*bx*qy-2.0f*bz*qw, -4.0f*bx*qx+2.0f*bz*qx},
        {-2.0f*bx*qz+2.0f*bz*qx, 2.0f*bx*qy+2.0f*bz*qw, 2.0f*bx*qx+2.0f*bz*qz, -2.0f*bx*qw+2.0f*bz*qy},
        {2.0f*bx*qy, 2.0f*bx*qz-4.0f*bz*qx, 2.0f*bx*qw-4.0f*bz*qy, 2.0f*bx*qx}
    };

    return J;
};

Eigen::Vector<float, 4> Madgwick::quatMultiply(const Eigen::Vector<float, 4> &q1, const Eigen::Vector<float, 4> &q2) {
    
    float w1 = q1(0);
    float i1 = q1(1);
    float j1 = q1(2);
    float k1 = q1(3);

    float w2 = q2(0);
    float i2 = q2(1);
    float j2 = q2(2);
    float k2 = q2(3);

    Eigen::Vector<float, 4> product;

    product(0) = w1 * w2 - i1 * i2 - j1 * j2 - k1 * k2;
    product(1) = w1 * i2 + i1 * w2 + j1 * k2 - k1 * j2;
    product(2) = w1 * j2 - i1 * k2 + j1 * w2 + k1 * i2;
    product(3) = w1 * k2 + i1 * j2 - j1 * i2 + k1 * w2;

    return product;
}

Eigen::Vector<float, 4> Madgwick::quatConj(const Eigen::Vector<float, 4> &q1) {

    Eigen::Vector<float, 4> conj {q1(0), -q1(1), -q1(2), -q1(3)};

    return conj;
}