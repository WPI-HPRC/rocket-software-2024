#include "State.h"
#include "Launch.h"
#include "Sensors.h"

Launch::Launch(struct Sensors *sensors, StateEstimator *stateEstimator, FlashChip *flashChip) : State(sensors, stateEstimator, flashChip) {}

void Launch::initialize_impl() {}

void Launch::loop_impl() {}

State *Launch::nextState_impl() {
	return nullptr;
}
