#pragma once
#include "State.h"
#include "Sensors.h"

class DrogueDescent : public State {
	_STATE_CLASS_IMPLS_
	public:
		DrogueDescent(struct Sensors *sensors, StateEstimator *stateEstimator, FlashChip *flashChip);
};
