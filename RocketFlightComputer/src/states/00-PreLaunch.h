#pragma once
#include "State.h"
#include "Debouncer.h"

class PreLaunch : public State {
	_STATE_CLASS_IMPLS_
	public:
		PreLaunch(struct Sensors *sensors, AttitudeStateEstimator *attitudeStateEstimator, KinematicStateEstimator *kinematicStateEstimator);
	private:
		float accelReadingBuffer[10] = {0};
		uint8_t buffIdx = 0;
		float avgAccelZ();
    bool launched = false;
    Debouncer launchDebouncer = Debouncer(30);
};
