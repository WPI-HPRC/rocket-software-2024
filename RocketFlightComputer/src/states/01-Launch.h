#pragma once
#include "State.h"
#include "Debouncer.h"

class Launch : public State
{
    _STATE_CLASS_IMPLS_
public:
    Launch(struct Sensors *sensors, AttitudeStateEstimator *attitudeStateEstimator, KinematicStateEstimator *kinematicStateEstimator);

private:
    bool motorBurnout = false;
    float accelerationBuffer[10] = {0};
    int bufferIndex = 0;
    Debouncer motorBurnoutDebouncer = Debouncer(30);
};
