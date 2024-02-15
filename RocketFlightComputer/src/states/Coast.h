#pragma once
#include "State.h"
#include "Sensors.h"

// seconds, OpenRocket for Test Launch 2/17
#define TIME_IN_COAST 19 * 1000

class Coast : public State
{
    _STATE_CLASS_IMPLS_
public:
    Coast(struct Sensors *sensors, StateEstimator *stateEstimator);

private:
    boolean apogeePassed = false;
    float transitionBufVelVert[10] = {0};
    int transitionBufIndVelVert = 0;
    float lastAltitude = 0;
};
