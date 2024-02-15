#include "MainDescent.h"
#include "Abort.h"
#include "Recovery.h"
#include "State.h"
#include "Sensors.h"

MainDescent::MainDescent(struct Sensors *sensors, StateEstimator *stateEstimator) : State(sensors, stateEstimator) {}

void MainDescent::initialize_impl() {}

void MainDescent::loop_impl() {
    float curr_altitude = sensorPacket.altitude;
    altitudeBuffer[altitudeBufferIndex] = curr_altitude;
    
    // TODO: Use the debouncer instead
    if (altitudeBufferIndex == 9) {
        // take the average of the full buffer
        float avg_sum = 0.0;
        float average = 0.0;
        for (int i = 0; i < 10; i++)
        {
            avg_sum += altitudeBuffer[i];
        }
        average = avg_sum / 10.0;

        // find average standard deviation
        // TODO: this is not the correct way to calculate standard deviation, fix this?
        float avg_std_sum = 0.0;
        float avg_std = 0.0;
        for (int i = 0; i < 10; i++)
        {
            avg_std_sum += abs(altitudeBuffer[i] - average);
        }
        avg_std = avg_std_sum / 10.0;
    }

    altitudeBufferIndex = (altitudeBufferIndex + 1) % 10;
}

State *MainDescent::nextState_impl() {
    if (/*avg_std < 2 && */sensorPacket.altitude <= LAND_THRESHOLD) {
        return new Recovery(sensors, stateEstimator);
    }

    // if the state hasn't changed for much more than the expected MAIN_DESCENT time, go to abort
    // 1.1 * TIME_IN_MAIN_DESCENT == 96.8 seconds
	if (this->currentTime > 1.1 * TIME_IN_MAIN_DESCENT) {
        return new Abort(sensors, stateEstimator);
    }

    return nullptr;
}

enum StateId MainDescent::getId() {
    return StateId::ID_MainDescent;
}
