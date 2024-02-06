#pragma once
#include "State.h"
#include "Sensors.h"

#define MAX_COAST_TIME 30000
class Coast : public State
{
    _STATE_CLASS_IMPLS_
public:
    Coast(struct Sensors *sensors, StateEstimator *stateEstimator, FlashChip *flashChip);

private:
    boolean apogeePassed = false;
};
