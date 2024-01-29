#include <Arduino.h>
#include "Recovery.h"
#include "State.h"
#include "Sensors.h"

Recovery::Recovery(struct Sensors *sensors) : State(sensors) {}

void Recovery::initialize_impl() {}

void Recovery::loop_impl() {
  // we want to run loop in a "low power mode", so just busy wait here
  delay(500);
}

State *Recovery::nextState_impl() {
	return nullptr;
}
