#pragma once
#include "State.h"
#include "Debouncer.h"

class Coast : public State
{
    _STATE_CLASS_IMPLS_
public:
    Coast(struct Sensors *sensors, AttitudeStateEstimator *attitudeStateEstimator);
    ~Coast();

private:
    boolean apogeePassed = false;
    float verticalVelocityBuffer[10] = {0};
    int bufferIndex = 0;
    float lastAltitude = 0;
    Debouncer apogeeDebouncer = Debouncer(10);

    enum AirbrakeServoState {
        WAIT = 0,
        FIRST_STEP,
        SECOND_STEP,
        DONE,
    } servoState = WAIT;
    int lastTransitionTime = 0;
};
