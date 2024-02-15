#pragma once
#include "State.h"
#include "Sensors.h"
// #include <controls/ekf/KalmanFilter.h>

class Debug : public State {
	_STATE_CLASS_IMPLS_
	public:
		Debug(struct Sensors *sensors, StateEstimator * ekf);
};
