#pragma once
#include "State.h"
#include "Sensors.h"

class MainDescent : public State {
	_STATE_CLASS_IMPLS_
	public:
		MainDescent(struct Sensors *sensors);
};
