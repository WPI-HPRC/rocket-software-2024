#include "Abort.h"
#include "State.h"

Abort::Abort(struct Sensors *sensors, StateEstimator *stateEstimator, FlashChip *flashChip) : State(sensors, stateEstimator, flashChip) {}

void Abort::initialize_impl() {}

void Abort::loop_impl()
{}

State *Abort::nextState_impl()
{
    return nullptr;
}
