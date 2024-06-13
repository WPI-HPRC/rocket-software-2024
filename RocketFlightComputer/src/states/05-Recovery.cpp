#include <Arduino.h>
#include "05-Recovery.h"
#include "State.h"

Recovery::Recovery(struct Sensors *sensors, AttitudeStateEstimator *attitudeStateEstimator) : State(sensors, attitudeStateEstimator) {}

void Recovery::initialize_impl() {}

void Recovery::loop_impl() {
  // we want to run loop in a "low power mode", so just busy wait here
	// actually maybe not, idk what delay will do to the rest of are code and I'm afraid of it
  // delay(500);
}

State *Recovery::nextState_impl() {
	return nullptr;
}

enum StateId Recovery::getId() {
	return StateId::ID_Recovery;
}
