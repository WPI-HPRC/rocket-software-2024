#include "ApogeeEstimation.h"

ApogeeEstimation::ApogeeEstimation() {
    Serial.println("[Apogee Estimator] Initialized!");
};

float ApogeeEstimation::estimate(BLA::Matrix<10> currentState, Utility::TelemPacket telemPacket) {

    BLA::Matrix<13> x_t = {
        currentState(0), // qw
        currentState(1), // qi
        currentState(2), // qj
        currentState(3), // qk
        currentState(7), // px
        currentState(8), // py
        currentState(9), // pz
        0, // omega_x
        0, // omega_y
        0, // omega_z
        currentState(4), // vx
        currentState(5), // vy
        currentState(6) // vz
    };

    float rk4Vel = 0.0f;

    // Loop until the N_z velocity of the vehicle begins to drop
    for(int i = 1; rk4Vel <= 0; i++) {

        BLA::Matrix<13> k1 = systemDynamics(x_t)*dt;
        BLA::Matrix<13> k2 = systemDynamics(x_t + k1*0.5f)*dt;
        // BLA::Matrix<13> k3 = systemDynamics(x_t + k2*0.5f)*dt;
        // BLA::Matrix<13> k4 = systemDynamics(x_t + k3)*dt;

        x_t = x_t + k1*0.1667f + k2*0.3333f;

        // x_t = x_t + k1*0.1667f + k2*0.3333f + k3*0.3333f + k4*0.1667f;

        rk4Vel = x_t(12); // Z Velocity from Estimate
    }

    return x_t(6);
}

BLA::Matrix<13> ApogeeEstimation::systemDynamics(BLA::Matrix<13> x) {

    BLA::Matrix<4> quat = {x(0), x(1), x(2), x(3)};

    BLA::Matrix<3,3> R_TB = Utility::quat2rotm(quat);

    BLA::Matrix<3> v_N = {x(10), x(11), x(12)};
    BLA::Matrix<3> v_hat = v_N;
    float v_NLen = BLA::Norm(v_N);
    if (v_NLen != 0) {
        v_hat /= v_NLen;
    }

    BLA::Matrix<3> v_B = R_TB*v_N;
    float dragForce = Utility::rho_sl*C_d*S_r* pow(BLA::Norm(v_B),2);

    float px_dot = (0 - dragForce * v_hat(0)) / (2*rocketMass);
    float py_dot = (0 - dragForce * v_hat(1)) / (2*rocketMass);
    float pz_dot = ((Utility::g*rocketMass) - dragForce * v_hat(2)) / (2*rocketMass);

    BLA::Matrix<13> x_dot = {
        0,
        0,
        0,
        0,
        x(10),
        x(11),
        x(12),
        0,
        0,
        0,
        px_dot,
        py_dot,
        pz_dot
    };

    return x_dot;

};