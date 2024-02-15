#pragma once
#include "State.h"
#include "Sensors.h"
#include "utility.hpp"

#define G_ACCEL 9.81
#define LAUNCH_ACCEL_THRESHOLD 3.0 * G_ACCEL

class PreLaunch : public State {
	_STATE_CLASS_IMPLS_
	public:
		PreLaunch(struct Sensors *sensors, StateEstimator *stateEstimator);
	private:
		bool stateEstimatorInitialized = false;
		float accelReadingBuffer[10];
		uint8_t buffIdx = 0;
		float avgAccelZ();
};
