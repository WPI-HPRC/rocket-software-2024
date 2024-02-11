#pragma once
#include "State.h"
#include "Sensors.h"

#define LAND_THRESHOLD 20 // Apparently landing height in meters
#define MAIN_DESCENT_TIMEOUT 100 * 1000 // 100 seconds

class MainDescent : public State {
	_STATE_CLASS_IMPLS_
    private:
        float altitudeBuffer[10];
	public:
		MainDescent(struct Sensors *sensors, StateEstimator *stateEstimator, FlashChip *flashChip);
};
