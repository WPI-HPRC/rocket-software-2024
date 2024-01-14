#pragma once
#include "State.h"
// #include <controls/ekf/KalmanFilter.h>
class Debug : public State {
	_STATE_CLASS_IMPLS_
	public:
		Debug(QuatStateEstimator * ekf);

    private:

        QuatStateEstimator * ekf;
};
