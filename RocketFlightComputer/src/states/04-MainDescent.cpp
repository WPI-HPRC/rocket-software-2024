#include "04-MainDescent.h"
#include "06-Abort.h"
#include "05-Recovery.h"
#include "State.h"
#include "Sensors.h"
#include "utility.hpp"

MainDescent::MainDescent(struct Sensors *sensors, StateEstimator *stateEstimator) : State(sensors, stateEstimator) {}

void MainDescent::initialize_impl() {}

void MainDescent::loop_impl()
{
    // calculate vertical velocity
    float verticalVelocity = (this->sensorPacket.altitude - this->lastAltitude) / (this->deltaTime / 1000.0);
    this->lastAltitude = this->sensorPacket.altitude;

    // add vertical velocity to cyclic buffer
    size_t buffLen = ARRAY_SIZE(this->verticalVelocityBuffer);
    Utility::circBufInsert(buffLen, this->verticalVelocityBuffer, &this->bufferIndex, verticalVelocity);

    // if the average vertical velocity is less than the expected landing velocity for 30 cycles, the rocket has landed
    this->landed = this->landedDebouncer.checkOut(Utility::average(buffLen, this->verticalVelocityBuffer) < LANDING_VELOCITY);
}

State *MainDescent::nextState_impl()
{
    // Transition state if condition met
    if (this->landed)
    {
        return new Recovery(this->sensors, this->stateEstimator);
    }

    // if the state hasn't changed for much more than the expected MAIN_DESCENT time, go to abort
    // 1.1 * TIME_IN_MAIN_DESCENT == 96.8 seconds
    if (this->currentTime > 1.1 * TIME_IN_MAIN_DESCENT)
    {
        return new Abort(this->sensors, this->stateEstimator);
    }

    return nullptr;
}

enum StateId MainDescent::getId()
{
    return StateId::ID_MainDescent;
}
