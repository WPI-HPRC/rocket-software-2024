#pragma once
#include "State.h"
#include "Sensors.h"

#define MOTOR_BURN_TIME 3 * 1000 // 3 second timeout

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
