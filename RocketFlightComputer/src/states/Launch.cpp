#include "State.h"
#include "Launch.h"
#include "Sensors.h"

Launch::Launch(struct Sensors *sensors) : State(sensors) {}

void Launch::initialize_impl() {}

void Launch::loop_impl() {
	// Stay in this state for at least 3 seconds to prevent airbrake activation
	logData();
	if (boostTimer.check() == 1)
	{
		boostTimerElapsed = true;
	}

	if (boostTimerElapsed)
	{
		if (motorBurnoutDetect())
		{
			state_start = millis();
		}
	}
}

State *Launch::nextState_impl() {
	// TODO: Figure out and implement the conditions for switch to Coast state
	// Switch to Coast when the conditions are right
	if (0) {
		return new Coast(sensors)
	}
	return nullptr;
}
