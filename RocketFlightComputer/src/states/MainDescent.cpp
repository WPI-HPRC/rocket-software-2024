#include "MainDescent.h"
#include "State.h"
#include "Sensors.h"

MainDescent::MainDescent(struct Sensors *sensors, StateEstimator *stateEstimator, FlashChip *flashChip) : State(sensors, stateEstimator, flashChip) {}

void MainDescent::initialize_impl() {}

void MainDescent::loop_impl() {
    float curr_altitude = sensorPacket.altitude;
    altitudeBuffer[altitudeBufferIndex] = curr_altitude;
    
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
    if (this->currentTime > MAIN_DESCENT_TIMEOUT && avg_std < 5 && curr_altitude <= LAND_THRESHOLD) {
        return new Recovery(sensors, stateEstimator, flash);
    }

	return nullptr;
}
