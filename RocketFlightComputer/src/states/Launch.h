#pragma once
#include "State.h"
#include "Sensors.h"

#define MIN_LAUNCH_TIME 3000 // TODO: check with OpenRocket sim

class Launch : public State
{
    _STATE_CLASS_IMPLS_
public:
    Launch(struct Sensors *sensors, StateEstimator *stateEstimator, FlashChip *flashChip);

private:
    bool motorBurnout = false;
    float transitionBufAcc[10] = {0};
    int transitionBufIndAcc = 0;
};
