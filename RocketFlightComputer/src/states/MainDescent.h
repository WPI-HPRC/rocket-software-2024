#pragma once
#include "State.h"
#include "Sensors.h"

// max height in meters to land
#define LAND_THRESHOLD 20
// 88 seconds, OpenRocket for Test Launch 2/17
#define TIME_IN_MAIN_DESCENT 88 * 1000

class MainDescent : public State
{
    _STATE_CLASS_IMPLS_
public:
    MainDescent(struct Sensors *sensors, StateEstimator *stateEstimator);

private:
    float altitudeBuffer[10];
    int altitudeBufferIndex = 0;
};
