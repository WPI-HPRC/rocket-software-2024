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
			avionicsState = COAST;
			break;
		}
	}
}

State *Launch::nextState_impl() {
	return nullptr;
}
