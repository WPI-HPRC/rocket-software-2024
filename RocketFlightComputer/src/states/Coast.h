#pragma once
#include "State.h"
#include "Sensors.h"
#include "Debouncer.h"

class Coast : public State
{
    _STATE_CLASS_IMPLS_
public:
    Coast(struct Sensors *sensors, StateEstimator *stateEstimator);

private:
    boolean apogeePassed = false;
    float verticalVelocityBuffer[10] = {0};
    int bufferIndex = 0;
    float lastAltitude = 0;
    Debouncer apogeeDebouncer = Debouncer(30);
};
