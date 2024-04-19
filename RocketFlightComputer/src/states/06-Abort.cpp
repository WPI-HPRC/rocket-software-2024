
// Abort state
// This state will close any control surfaces and slowly log data

#include "06-Abort.h"
#include "State.h"

Abort::Abort(struct Sensors *sensors, StateEstimator *stateEstimator) : State(sensors, stateEstimator) {}

void Abort::initialize_impl() {}

void Abort::loop_impl()
{
    // Just continue logging
}

State *Abort::nextState_impl()
{
    return nullptr;
}

enum StateId Abort::getId() {
    return StateId::ID_Abort;
}
