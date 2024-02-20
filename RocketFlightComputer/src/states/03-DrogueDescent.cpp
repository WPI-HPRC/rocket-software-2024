#include "State.h"
#include "03-DrogueDescent.h"
#include "04-MainDescent.h"
#include "06-Abort.h"
#include "Sensors.h"
#include "utility.hpp"

DrogueDescent::DrogueDescent(struct Sensors *sensors, StateEstimator *stateEstimator) : State(sensors, stateEstimator) {}

void DrogueDescent::initialize_impl() {}

// TODO: Debounce
void DrogueDescent::loop_impl()
{
    // calculate vertical velocity
    float verticalVelocity = (this->sensorPacket.altitude - this->lastAltitude) / (this->deltaTime / 1000.0);
    this->lastAltitude = this->sensorPacket.altitude;

    // add vertical velocity to cyclic buffer
    size_t buffLen = ARRAY_SIZE(this->verticalVelocityBuffer);
    Utility::circBufInsert(buffLen, this->verticalVelocityBuffer, &this->bufferIndex, verticalVelocity);

    // if the average vertical velocity is less that the expected velocity at main deploy for 30 cycles, main has deployed
    this->mainDeployVelocityReached = this->drogueDescentDebouncer.checkOut(Utility::average(buffLen, this->verticalVelocityBuffer) <= MAIN_DEPLOY_VELOCITY);
}

State *DrogueDescent::nextState_impl()
{
    // Transition state if condition met
    if (this->mainDeployVelocityReached)
    {
        return new MainDescent(this->sensors, this->stateEstimator);
    }

    // if the state hasn't changed for much more than the expected DROGUE time, go to abort
    // 1.2 * TIME_IN_DROGUE_DESCENT == 85.2 seconds
    if (this->currentTime > 1.2 * TIME_IN_DROGUE_DESCENT)
    {
        return new Abort(this->sensors, this->stateEstimator);
    }

    return nullptr;
}

enum StateId DrogueDescent::getId()
{
    return StateId::ID_DrogueDescent;
}
