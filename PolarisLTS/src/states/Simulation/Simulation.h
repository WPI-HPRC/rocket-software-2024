#pragma once
#include <states/State.h>
#include <Arduino.h>

class Simulation : public State {

    _STATE_CLASS_IMPLS_
    public:
    // Simulation(StateEstimator * stateEstimator);
    Simulation(StateEstimator * ekf);
    
    private:
    StateEstimator * ekf;
    // StateEstimator * ekf;

};