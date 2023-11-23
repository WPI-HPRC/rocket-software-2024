#include "Simulation.h"
#include <states/State.h>

Simulation::Simulation() {
    this->name = "Simulation";
};

void Simulation::initialize_impl() {

};

void Simulation::loop_impl() {
};

State *Simulation::nextState_impl() {

    return nullptr; // DO NOT EXIT STATE
};