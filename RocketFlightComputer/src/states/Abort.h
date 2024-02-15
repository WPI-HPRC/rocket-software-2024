#pragma once
#include "State.h"
#include "Sensors.h"
// #include <controls/ekf/KalmanFilter.h>
class Abort : public State
{
    _STATE_CLASS_IMPLS_
public:
    Abort(struct Sensors *sensors, StateEstimator *stateEstimator);
};
