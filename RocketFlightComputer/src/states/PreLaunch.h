#pragma once
#include "State.h"

//! @brief max number of milliseconds we can remain in the prelaunch (initialization) state
#define MAX_PRELAUNCH_TIME 3000
class PreLaunch : public State {
	_STATE_CLASS_IMPLS_
	public:
		PreLaunch();

	private:
		StateEstimator * ekf;
	
};
