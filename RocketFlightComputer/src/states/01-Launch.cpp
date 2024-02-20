#include "State.h"
#include "01-Launch.h"
#include "02-Coast.h"
#include "06-Abort.h"
#include "Sensors.h"
#include "utility.hpp"

Launch::Launch(struct Sensors *sensors, StateEstimator *stateEstimator) : State(sensors, stateEstimator) {}

void Launch::initialize_impl() {}

void Launch::loop_impl()
{
    // detect motor burnout via acceleration
    // add current Z acceleration to cyclic buffer
    size_t buffLen = ARRAY_SIZE(this->accelerationBuffer);
    Utility::circBufInsert(buffLen, this->accelerationBuffer, &this->bufferIndex, this->sensorPacket.accelZ);

    // if the average Z acceleration is less than the burnout threshold for 30 cycles, motor has burned out
    this->motorBurnout = this->motorBurnoutDebouncer.checkOut(Utility::average(buffLen, this->accelerationBuffer) < BURNOUT_THRESHOLD);
}

State *Launch::nextState_impl()
{
    // Stay in this state for at least 3 seconds to prevent airbrake activation under motor power
    if (this->currentTime >= MOTOR_BURN_TIME && this->motorBurnout)
    {
        return new Coast(this->sensors, this->stateEstimator);
    }

    // if state hasn't changed for much more than 2x motor burnout time, go to abort
    if (this->currentTime > 2 * MOTOR_BURN_TIME)
    {
        return new Abort(this->sensors, this->stateEstimator);
    }

    return nullptr;
}

enum StateId Launch::getId()
{
    return StateId::ID_Launch;
}
