#pragma once
#include <states/State.h>
#include <Arduino.h>
// #include <Controls/EKF/KalmanFilter.h>
#include <ControlsOld/EKF/StateEstimator.h>

#define MAX_PRELAUNCH_TIME 3000
class PreLaunch : public State
{
    _STATE_CLASS_IMPLS_
public:
    PreLaunch();

private:
    // StateEstimator * ekf;
    QuatStateEstimator * ekf;

};