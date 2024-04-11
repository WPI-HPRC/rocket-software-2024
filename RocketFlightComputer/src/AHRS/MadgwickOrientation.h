#pragma once

#include <Arduino.h>

#include <ArduinoEigenSparse.h>
#include <utility.hpp>

class Madgwick {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Madgwick(const Eigen::Vector<float, 4> &q0, float gain);

        // BLA::Matrix<4> onLoop(Utility::SensorPacket sensorPacket);
        Eigen::Vector<float, 4> onLoop(Utility::SensorPacket sensorPacket);

    private:
        float dt = 1.0 / LOOP_RATE;

        Eigen::Vector<float, 4> quatMultiply(const Eigen::Vector<float, 4> &q_1, const Eigen::Vector<float, 4> &q_2);

        Eigen::Vector<float, 4> quatConj(const Eigen::Vector<float, 4> &q1);

        Eigen::Vector<float, 6> updateFunction(const Eigen::Vector<float, 3> &acc, const Eigen::Vector<float, 3> &mag);

        Eigen::Matrix<float, 6,4> updateJacobian(const Eigen::Vector<float, 3> &acc, const Eigen::Vector<float, 3> &mag);

        Eigen::Vector<float, 4> q;
        float gain = 0.0f;

};