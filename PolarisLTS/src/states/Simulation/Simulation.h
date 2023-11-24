#pragma once
#include <states/State.h>
#include <Arduino.h>

class Simulation : public State {

    _STATE_CLASS_IMPLS_
    public:
        Simulation(QuatStateEstimator estimator);
    
    private:

    QuatStateEstimator * stateEstimator;

};