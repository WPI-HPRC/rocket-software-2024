#pragma once
#include "State.h"
#include "Sensors.h"

#define MAX_LAUNCH_TIME 3000

class Launch : public State {
	_STATE_CLASS_IMPLS_
	public:
		Launch(struct Sensors *sensors);
	private:
		bool motorBurnout = false;
};
