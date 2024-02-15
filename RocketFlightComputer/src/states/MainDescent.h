#pragma once
#include "State.h"
#include "Sensors.h"
#include "Debouncer.h"

class MainDescent : public State
{
    _STATE_CLASS_IMPLS_
public:
    MainDescent(struct Sensors *sensors, StateEstimator *stateEstimator);

private:
    float transitionBufVelVert[10] = {0};
    int transitionBufIndVelVert = 0;
    float lastAltitude = 0;
    bool landed = false;
    Debouncer landedDebouncer = Debouncer(30);
};
