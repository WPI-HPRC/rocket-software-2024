#pragma once
#include "Servo.h"
#include "State.h"
#include "Debouncer.h"

class Coast : public State
{
    _STATE_CLASS_IMPLS_
public:
    Coast(struct Sensors *sensors, StateEstimator *stateEstimator);
    ~Coast();

private:
    boolean apogeePassed = false;
    float verticalVelocityBuffer[10] = {0};
    int bufferIndex = 0;
    float lastAltitude = 0;
    Debouncer apogeeDebouncer = Debouncer(30);
    Servo airbrakesServo = Servo();

    enum AirbrakeServoState {
        WAIT,
        EXTEND_FULL,
        EXTEND_HALF,
        SWEEP_FORWARD,
        SWEEP_BACKWARD,
    } servoState;
};
