#include "MainDescent.h"
#include "State.h"
#include "Sensors.h"

MainDescent::MainDescent(struct Sensors *sensors) : State(sensors) {}

void MainDescent::initialize_impl() {}

void MainDescent::loop_impl() {}

State *MainDescent::nextState_impl() {
	return nullptr;
}
