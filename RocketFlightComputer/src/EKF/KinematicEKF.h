#pragma once

#include <Arduino.h>
#include <BasicLinearAlgebra.h>
#include "../../src/utility.hpp"
#include "../../src/FlightParams.hpp"

/**
 * 
 * @brief Position and Velocity state estmiator fusing GPS, Barometer and Accelerometer
 * @author @frostydev99 - Daniel Pearson
 */
class KinematicStateEstimator {
    public:

    KinematicStateEstimator();

    void init(BLA::Matrix<6> x_0, float h_0, float dt);

    void onLoop(Utility::TelemPacket sensorPacket);

    BLA::Matrix<3,3> quat2rotm(BLA::Matrix<4> q);

    BLA::Matrix<6> x{};

    bool initialized = false;
    float dt = 0.025;

    private:

    const float accelXY_Var = 0.00637; // [m/s^2]
    const float accelZ_Var = 0.00686; // [m/s^2]
    const float gyroVar = 0.000489; // [rad/s]
    const float baroVar = 1; // [Pa]
    float initialHeight = 0.0;

    BLA::Matrix<6,6> eye6 = {
        1,0,0,0,0,0,
        0,1,0,0,0,0,
        0,0,0,1,0,0,
        0,0,0,0,1,0,
        0,0,0,0,0,1
    };

    BLA::Matrix<6,6> P = eye6;

    BLA::Matrix<6,6> P_min = eye6;

    BLA::Matrix<7,7> R = {
        1,0,0,0,0,0,0,
        0,1,0,0,0,0,0,
        0,0,1,0,0,0,0,
        0,0,0,1,0,0,0,
        0,0,0,0,1,0,0,
        0,0,0,0,0,1,0,
        0,0,0,0,0,0,1
    };

    /**
     * @brief Measurement function predicting vehicle dynamics
     * 
     * @param u <AxN, AyN, AyZ>
     * @return BLA::Matrix<7> 
     */
    BLA::Matrix<6> measurementFunction(BLA::Matrix<3> u);

    /**
     * @brief Jacobian of measurement function
     * 
     * @param u <AxN, AyN, AyZ>
     * @return BLA::Matrix<6,6> 
     */
    BLA::Matrix<6,6> measurementJacobian(BLA::Matrix<3> u);

    /**
     * @brief Correction Step Function
     * 
     * @param z <GpsLat, GpsLon, GpsAlt, VelocityN, VelocityE, VelocityD, BaroPressure>
     * @return BLA::Matrix<7> 
     */
    BLA::Matrix<7> updateFunction(BLA::Matrix<7> z);

    /**
     * @brief Jacobian of correction step
     * 
     * @param z <GpsLat, GpsLon, GpsAlt, Gps VelX, Gps VelY, Gps VelZ, BaroPressure>
     * @return BLA::Matrix<7,7> 
     */
    BLA::Matrix<7,6> updateJacobian(BLA::Matrix<7> z);

    BLA::Matrix<6> x_min;

    constexpr static float g = Utility::g;
    BLA::Matrix<3> gravT = {0,0,g};

};