#include "State.h"
#include "03-DrogueDescent.h"
#include "04-MainDescent.h"
#include "06-Abort.h"
#include "Sensors.h"

DrogueDescent::DrogueDescent(struct Sensors *sensors, StateEstimator *stateEstimator) : State(sensors, stateEstimator) {}

void DrogueDescent::initialize_impl() {}

// TODO: Debounce
void DrogueDescent::loop_impl()
{
    // calculate vertical velocity
    float verticalVelocity = (sensorPacket.altitude - lastAltitude) / (deltaTime / 1000.0);
    lastAltitude = sensorPacket.altitude;

    // add vertical velocity to cyclic buffer
    verticalVelocityBuffer[bufferIndex] = verticalVelocity;

    // average all values in the buffer
    float sum = 0.0;
    float averageVerticalVelocity = 0.0;
    for (int i = 0; i < 10; i++)
    {
        sum += verticalVelocityBuffer[i];
    }
    averageVerticalVelocity = sum / 10.0;

    bufferIndex = (bufferIndex + 1) % 10;

    // if the average vertical velocity is less that the expected velocity at main deploy for 30 cycles, main has deployed
    mainDeployVelocityReached = drogueDescentDebouncer.checkOut(averageVerticalVelocity <= MAIN_DEPLOY_VELOCITY);
}

State *DrogueDescent::nextState_impl()
{
    // Transition state if condition met
    if (mainDeployVelocityReached)
    {
        return new MainDescent(sensors, stateEstimator);
    }

    // if the state hasn't changed for much more than the expected DROGUE time, go to abort
    // 1.2 * TIME_IN_DROGUE_DESCENT == 85.2 seconds
    if (this->currentTime > 1.2 * TIME_IN_DROGUE_DESCENT)
    {
        return new Abort(sensors, stateEstimator);
    }

    return nullptr;
}

enum StateId DrogueDescent::getId()
{
    return StateId::ID_DrogueDescent;
}
