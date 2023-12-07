#include "Coast.h"
#include "State.h"
#include "Launch.h"
#include "GNSS.h"

Coast::Coast() {}

void Coast::initialize_impl() {
	gnss.initialize();
}

void Coast::loop_impl() {}

State *Coast::nextState_impl() {
	if (gnss.getLatitude() == 100) {
		return new Launch();
	}
	return nullptr;
}

StateId Coast::getId() {
	return StateId::Coast;
}
