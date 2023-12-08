#include "State.h"
#include "DrogueDescent.h"


DrogueDescent::DrogueDescent() {}

void DrogueDescent::initialize_impl() {}

void DrogueDescent::loop_impl() {}

State *DrogueDescent::nextState_impl() {
	return nullptr;
}

StateId DrogueDescent::getId() {
	return StateId::DrogueDescent;
}
