#include "State.h"
#include "DrogueDescent.h"
#include "Sensors.h"

DrogueDescent::DrogueDescent(struct Sensors *sensors) {
	this->sensors = sensors;
}

void DrogueDescent::initialize_impl() {}

void DrogueDescent::loop_impl() {}

State *DrogueDescent::nextState_impl() {
	return nullptr;
}
