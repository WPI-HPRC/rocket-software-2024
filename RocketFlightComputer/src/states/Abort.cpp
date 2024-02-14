#include "Abort.h"
#include "State.h"

Abort::Abort(struct Sensors *sensors, StateEstimator *stateEstimator) : State(sensors, stateEstimator) {}

void Abort::initialize_impl() {}

void Abort::loop_impl()
{
    // shut down all systems, log slowly

    // busy wait
    delay(500);
}

State *Abort::nextState_impl()
{
    return nullptr;
}

enum StateId Abort::getId() {
    return StateId::ID_Abort;
}
