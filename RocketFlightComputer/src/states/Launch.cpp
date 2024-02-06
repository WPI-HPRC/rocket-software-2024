#include "State.h"
#include "Launch.h"
#include "Coast.h"
#include "Sensors.h"

Launch::Launch(struct Sensors *sensors, StateEstimator *stateEstimator, FlashChip *flashChip) : State(sensors, stateEstimator, flashChip) {}

void Launch::initialize_impl() {}

void Launch::loop_impl() {
	// old motorBurnoutDetect code
	// accel value gets updated in sensor reading fcn
    // add to cyclic buffer
    transitionBufAcc[transitionBufIndAcc] = sensorPacket.ac_z;
    // take running average value
    float sum = 0.0;
    for (int i = 0; i < 10; i++)
    {
        sum += transitionBufAcc[i];
    }
    sum = sum / 10.0;

    transitionBufIndAcc = (transitionBufIndAcc + 1) % 10;
    // compare running average value to defined threshold
    if (sum < 0)
    {
        for (int j = 0; j < 10; j++)
        {
            transitionBufAcc[j] = 0;
        }
        transitionBufIndAcc = 0;
        Serial.println("Motor burnout detected!");
    }
}

State *Launch::nextState_impl() {
	// Stay in this state for at least 3 seconds to prevent airbrake activation
	//Switch to Coast when the conditions are right (TODO: fix this shitty comment)
	if (this->currentTime > MAX_LAUNCH_TIME && motorBurnout) {
		return new Coast(sensors);
	}
	return nullptr;
}
