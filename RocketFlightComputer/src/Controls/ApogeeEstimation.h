#pragma once

#include <Arduino.h>
#include <utility.hpp>
#include <FlightParams.hpp>

class ApogeeEstimation {

public:

    ApogeeEstimation();
    
    float estimate(BLA::Matrix<10> currentState, Utility::TelemPacket telemPacket);

private:

    // RK4 Constants
    const float dt = 5; // 1Hz search
    const float t_0 = 0;
    const float t_max = 50; // [s] 50s max forward look
    const float numTimePts = t_max / dt+1;

    BLA::Matrix<13> systemDynamics(BLA::Matrix<13> x);
};