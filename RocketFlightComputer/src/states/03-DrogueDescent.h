#pragma once
#include "State.h"
#include "Sensors.h"
#include "Debouncer.h"

class DrogueDescent : public State
{
    _STATE_CLASS_IMPLS_
public:
    DrogueDescent(struct Sensors *sensors, StateEstimator *stateEstimator);

private:
    boolean mainDeployVelocityReached = false;
    float verticalVelocityBuffer[10] = {0};
    int bufferIndex = 0;
    float lastAltitude = 0;
    Debouncer drogueDescentDebouncer = Debouncer(30);
};
