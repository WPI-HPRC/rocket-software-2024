#pragma once
#include "State.h"
#include "Sensors.h"
#include "Debouncer.h"

class PreLaunch : public State {
	_STATE_CLASS_IMPLS_
	public:
		PreLaunch(struct Sensors *sensors, StateEstimator *stateEstimator);
	private:
		float accelerationBuffer[10] = {0};
		size_t bufferIndex = 0;
    bool launched = false;
    Debouncer launchDebouncer = Debouncer(30);
};
