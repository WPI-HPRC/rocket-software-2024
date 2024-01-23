#include "Coast.h"
#include "State.h"
#include "Sensors.h"

Coast::Coast(struct Sensors *sensors) {
	this->sensors = sensors;
}

void Coast::initialize_impl() {}

void Coast::loop_impl() {}

State *Coast::nextState_impl() {
	return nullptr;
}
