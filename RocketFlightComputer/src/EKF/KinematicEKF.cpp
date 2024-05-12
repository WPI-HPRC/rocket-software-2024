#include "KinematicEKF.h"

KinematicStateEstimator::KinematicStateEstimator() = default;

void KinematicStateEstimator::init(BLA::Matrix<6> x_0, float h_0, float dt) {
    this->x = x_0;
    this->x_min = x_0;
    this->dt = dt;
    this->initialized = true;
    this->initialHeight = h_0;
}

void KinematicStateEstimator::onLoop(Utility::TelemPacket telemPacket) {
    float accX = telemPacket.accelX * g;
    float accY = telemPacket.accelY * g;
    float accZ = telemPacket.accelZ * g;

    // Convert pressure from mbar to Pascals
    float baroAlt = telemPacket.pressure * 100; // [Pa]

    float gpsLat = telemPacket.gpsLat / 1e7;
    float gpsLong = telemPacket.gpsLong / 1e7;
    float gpsAlt = telemPacket.gpsAltMSL / 1e2;

    

    BLA::Matrix<4> q = {telemPacket.w, telemPacket.i, telemPacket.j, telemPacket.k};
    BLA::Matrix<3> accelR = {accX, accY, accZ};

    BLA::Matrix<3,3> rotm = quat2rotm(q);

    BLA::Matrix<3> accelT = rotm * accelR;

    BLA::Matrix<3> linAccelT = accelT + gravT;

    x_min = measurementFunction(linAccelT);

    BLA::Matrix<6,6> A = measurementJacobian(linAccelT);

    BLA::Matrix<7> z = {gpsLat, gpsLong, gpsAlt, telemPacket.gpsVelocityN, telemPacket.gpsVelocityE, telemPacket.gpsVelocityD, telemPacket.pressure};

    BLA::Matrix<7> y = {x(0), x(1), x(2), x(3), x(4), x(5), x(2)};

    BLA::Matrix<7> h = updateFunction(z);

    BLA::Matrix<7,6> H = updateJacobian(z);

    BLA::Matrix<7> v = y - h;
    BLA::Matrix<7,7> S = H * P_min * BLA::MatrixTranspose<BLA::Matrix<7, 6>>(H) + R;
    BLA::Matrix<6,7> K = P_min * BLA::MatrixTranspose<BLA::Matrix<7,6>>(H) * BLA::Inverse(S);

    x = x_min + K*v;

    P = (eye6 - K*H) * P_min;

}

BLA::Matrix<6> KinematicStateEstimator::measurementFunction(BLA::Matrix<3> u) {

    BLA::Matrix<6> inputVector = {u(0), u(1), u(2)};

    BLA::Matrix<6,6> AMatrix = {
            1, 0, 0, dt, 0, 0,
            0, 1, 0, 0, dt, 0,
            0, 0, 1, 0, 0, dt,
            0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 1
    };

    BLA::Matrix<6,3> BMatrix = {
        0.5f*(dt*dt), 0, 0,
        0, 0.5f*(dt*dt), 0,
        0, 0, 0.5f*(dt*dt),
        dt, 0, 0,
        0, dt, 0,
        0, 0, dt
    };

    BLA::Matrix<6> x_dot = AMatrix * x + BMatrix * u;
    return x_dot;
}

BLA::Matrix<6, 6> KinematicStateEstimator::measurementJacobian(BLA::Matrix<3> u) {
    
    BLA::Matrix<6,6> J = {
        1, 0, 0, dt, 0, 0,
        0, 1, 0, 0, dt, 0,
        0, 0, 1, 0, 0, dt,
        0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1
    };

    return J;
}

BLA::Matrix<7> KinematicStateEstimator::updateFunction(BLA::Matrix<7> z) {

    float r_adj = Utility::r_earth + z(2); // [m]
    
    float R_N = Utility::a_earth / sqrt(1 - pow(Utility::e_earth, 2)*pow(sin(z(0) * DEG_TO_RAD), 2));

    float X = (R_N + r_adj)*cos(z(0) * DEG_TO_RAD)*cos(z(1) * DEG_TO_RAD);
    float Y = (R_N + r_adj)*cos(z(0) * DEG_TO_RAD)*sin(z(1) * DEG_TO_RAD);
    float Z = ((1-pow(Utility::e_earth, 2)) * R_N + r_adj)*sin(z(0) * DEG_TO_RAD);

    float h = 44330 * (1 - pow((z(6) / Utility::P_sl), 0.19)) - initialHeight;

    BLA::Matrix<7> hf = {
        X,
        Y,
        Z,
        z(3),
        z(4),
        z(5),
        h
    };

    // 
        // float N_earth = Utility::a_earth / sqrt(1 - pow(Utility::e_earth, 2) * pow(sin(sensorPacket.gpsLat), 2));

        // float X_0 = (N_earth + sensorPacket.gpsAltAGL) * cos(sensorPacket.gpsLat * DEG_TO_RAD) * cos(sensorPacket.gpsLong * DEG_TO_RAD);
        // float Y_0 = (N_earth + sensorPacket.gpsAltAGL) * cos(sensorPacket.gpsLat * DEG_TO_RAD) * sin(sensorPacket.gpsLong * DEG_TO_RAD);
        // float Z_0 = (((Utility::b_earth * Utility::b_earth) / (Utility::a_earth * Utility::a_earth)) * N_earth + sensorPacket.gpsAltAGL) * sin(sensorPacket.gpsLat * DEG_TO_RAD);
        // float Z_0 = (N_earth*(1-pow(Utility::e_earth,2))+sensorPacket.gpsAltAGL)*sin(sensorPacket.gpsLat);
    
    return hf;
}

BLA::Matrix<7, 6> KinematicStateEstimator::updateJacobian(BLA::Matrix<7> y) {
    return 0;
}

BLA::Matrix<3, 3> KinematicStateEstimator::quat2rotm(BLA::Matrix<4> q)
{

    float qw = q(0);
    float qx = q(1);
    float qy = q(2);
    float qz = q(3);

    BLA::Matrix<3, 3> rotm = {
        qw*qw + qx*qx - qy*qy - qz*qz, 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy),
        2 * (qx * qy + qw * qz), qw*qw - qx*qx + qy*qy - qz*qz, 2 * (qy * qz - qw * qx),
        2 * (qx * qz - qw * qy), 2 * (qw * qx + qy * qz), qw*qw - qx*qx - qy*qy + qz*qz
    };

    return rotm;
};