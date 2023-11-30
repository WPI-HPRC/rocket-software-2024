#include "Simulation.h"
#include <states/State.h>

Simulation::Simulation(StateEstimator * stateEstimator) {
    this->name = "Simulation";
    this->ekf = stateEstimator;
};

void Simulation::initialize_impl() {

};

void Simulation::loop_impl() {
    this->currentState = ekf->onLoop(sensorData);
    
};

State *Simulation::nextState_impl() {

    return nullptr; // DO NOT EXIT STATE
};