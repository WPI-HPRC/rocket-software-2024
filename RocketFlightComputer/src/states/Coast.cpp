#include "Coast.h"
#include "State.h"
#include "Sensors.h"

Coast::Coast(struct Sensors *sensors, StateEstimator *stateEstimator, FlashChip *flashChip) : State(sensors, stateEstimator, flashChip) {}

void Coast::initialize_impl() {}

void Coast::loop_impl() {}

State *Coast::nextState_impl() {
	return nullptr;
}
