#pragma once
#include "State.h"
#include "Sensors.h"

#define LAND_THRESHOLD 20               // max height in meters to land
#define MAIN_DESCENT_TIMEOUT 100 * 1000 // 100 seconds

class MainDescent : public State
{
    _STATE_CLASS_IMPLS_
public:
    MainDescent(struct Sensors *sensors, StateEstimator *stateEstimator, FlashChip *flashChip);

private:
    float altitudeBuffer[10];
    int altitudeBufferIndex = 0;
};
