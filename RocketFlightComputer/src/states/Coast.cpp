#include "Coast.h"
#include "State.h"
#include "Launch.h"
#include "GNSS.h"
#include <cstdio>

Coast::Coast(GNSS gnss) {
	this->gnss = gnss;
}

void Coast::initialize_impl() {
	gnss.initialize();
}

void Coast::loop_impl() {}

State *Coast::nextState_impl() {
	printf("next_state\n");
	if (gnss.getLatitude() == 100) {
	 return new Launch();
	}
	return nullptr;
	// return new Launch();
}

StateId Coast::getId() {
	return StateId::Coast;
}
