#include "PreLaunch.h"
#include "State.h"
#include "Launch.h"
#include "Debug.h"

PreLaunch::PreLaunch() {
}

void PreLaunch::initialize_impl() {
	// Initialize sensors
	// We **definitely** don't want to spin forever here, but it doesn't hurt to try multiple times if initializing fails at first
	Eigen::Vector<float, 10> x_0 = {1,0,0,0,0,0,0,0,0,0};
	ekf = new StateEstimator(x_0, 0.025);
}

void PreLaunch::loop_impl() {
	// Read bno for example
}

//! @details If we are separating this from `Launch`, we need a time limit on this state or something
State *PreLaunch::nextState_impl()
{

	if(DEBUG_MODE) {
		return new Debug(this->ekf);
	}

	if (this->currentTime > MAX_PRELAUNCH_TIME)
	{
		return new Launch();
	}
	return nullptr;
}
