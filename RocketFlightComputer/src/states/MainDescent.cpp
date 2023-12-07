#include "MainDescent.h"
#include "State.h"

MainDescent::MainDescent() {}

void MainDescent::initialize_impl() {}

void MainDescent::loop_impl() {}

State *MainDescent::nextState_impl() {
	return nullptr;
}

StateId MainDescent::getId() {
	return StateId::MainDescent;
}