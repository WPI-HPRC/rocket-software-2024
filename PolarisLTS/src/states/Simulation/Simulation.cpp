#include "Simulation.h"
#include <states/State.h>
#include <ArduinoEigen.h>

Simulation::Simulation(QuatStateEstimator estimator) {
    this->name = "Simulation";
    this->stateEstimator = &estimator;
};

void Simulation::initialize_impl() {

};

void Simulation::loop_impl() {
    this->currentState = stateEstimator->onLoop(sensorData);

    // Serial.print("Vector4f: ");
    // Serial.print(currentState[0]);
    // Serial.print(", ");
    // Serial.print(currentState[1]);
    // Serial.print(", ");
    // Serial.print(currentState[2]);
    // Serial.print(", ");
    // Serial.println(currentState[3]);
};

State *Simulation::nextState_impl() {

    return nullptr; // DO NOT EXIT STATE
};