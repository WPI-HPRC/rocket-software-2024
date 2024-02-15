#pragma once
#include "State.h"
#include "Sensors.h"

class Launch : public State
{
    _STATE_CLASS_IMPLS_
public:
    Launch(struct Sensors *sensors, StateEstimator *stateEstimator);

private:
    bool motorBurnout = false;
    float transitionBufAcc[10] = {0};
    int transitionBufIndAcc = 0;
};
