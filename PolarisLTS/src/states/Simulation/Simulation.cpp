#include "Simulation.h"
#include <states/State.h>

Simulation::Simulation(QuatStateEstimator estimator) {
    this->name = "Simulation";
    this->stateEstimator = &estimator;
};

void Simulation::initialize_impl() {

};

void Simulation::loop_impl() {
    this->currentState = stateEstimator->onLoop(sensorData);
};

State *Simulation::nextState_impl() {

    return nullptr; // DO NOT EXIT STATE
};