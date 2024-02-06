#pragma once
#include "State.h"
#include "Sensors.h"

class PreLaunch : public State {
	_STATE_CLASS_IMPLS_
	public:
		PreLaunch(struct Sensors *sensors, StateEstimator *stateEstimator, FlashChip *flashChip);
	private:
		bool stateEstimatorInitialized = false;
		float accelReadingBuffer[10];
		uint8_t buffIdx = 0;
		float avgAccelZ();
};
