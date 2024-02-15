#pragma once
#include "State.h"
#include "Sensors.h"

#define FPS_TO_MPS 3.281
// converts ft/s to m/s, OpenRocket sim Test Launch 2/17
#define MAIN_DEPLOY_VELOCITY 84.7 / FPS_TO_MPS

#define TIME_IN_DROGUE_DESCENT 71 * 1000 // seconds, OpenRocket for Test Launch 2/17
class DrogueDescent : public State
{
    _STATE_CLASS_IMPLS_
public:
    DrogueDescent(struct Sensors *sensors, StateEstimator *stateEstimator);

private:
    boolean drogueDescentRateMatched = false;
    float transitionBuffVerticalVelocity[10] = {0};
    int transitionBuffIndexVerticalVelocity = 0;
    float lastAltitude = 0;
};
