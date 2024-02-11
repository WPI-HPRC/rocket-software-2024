#pragma once
#include "State.h"
#include "Sensors.h"

#Define DROGUE_DESCENT_RATE 10 // m/s

class DrogueDescent : public State
{
    _STATE_CLASS_IMPLS_
public:
    DrogueDescent(struct Sensors *sensors, StateEstimator *stateEstimator, FlashChip *flashChip);

private:
    boolean drogueDescentRateMatched = false;
    float transitionBuffVerticalVelocity[10] = {0};
    int transitionBuffIndexVerticalVelocity = 0;
    float lastAltitude = 0;
};
