#pragma once
#include "State.h"
#include "Sensors.h"

class Recovery : public State {
	_STATE_CLASS_IMPLS_
	public:
		Recovery(struct Sensors *sensors, StateEstimator *stateEstimator, FlashChip *flashChip);
};
