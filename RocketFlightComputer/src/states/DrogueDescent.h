#pragma once
#include "State.h"
#include "Sensors.h"

class DrogueDescent : public State
{
    _STATE_CLASS_IMPLS_
public:
    DrogueDescent(struct Sensors *sensors, StateEstimator *stateEstimator);

private:
    boolean drogueDescentRateMatched = false;
    float transitionBuffVerticalVelocity[10] = {0};
    int transitionBuffIndexVerticalVelocity = 0;
    float lastAltitude = 0;
};
