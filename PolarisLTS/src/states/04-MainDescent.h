#pragma once
#include "State.h"
#include "Debouncer.h"

class MainDescent : public State
{
    _STATE_CLASS_IMPLS_
public:
    MainDescent(Sensorboard *sensors, AttitudeStateEstimator *attitudeStateEstimator);

private:
    float verticalVelocityBuffer[10] = {0};
    int bufferIndex = 0;
    float lastAltitude = 0;
    bool landed = false;
    Debouncer landedDebouncer = Debouncer(10);
};
