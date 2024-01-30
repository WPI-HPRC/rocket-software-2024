#pragma once
#include "State.h"
#include "Sensors.h"

class Coast : public State {
	_STATE_CLASS_IMPLS_
	public:
		Coast(struct Sensors *sensors);
};
