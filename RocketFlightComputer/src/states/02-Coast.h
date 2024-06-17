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
        FULL = 1,
        THREE_QUARTERS = 2,
        HALF = 3,
        ONE_QUARTER = 4,
        RETRACTED = 5,
    } servoState = WAIT;
};
