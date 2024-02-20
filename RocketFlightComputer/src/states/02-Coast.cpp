
// Coast state
// This state occurs after the motor has burned out as the rocket is on its way to apogee

#include "02-Coast.h"
#include "03-DrogueDescent.h"
#include "06-Abort.h"
#include "State.h"
#include "Sensors.h"
#include "utility.hpp"

Coast::Coast(struct Sensors *sensors, StateEstimator *stateEstimator) : State(sensors, stateEstimator) {}

void Coast::initialize_impl() {}

void Coast::loop_impl()
{
    // calculate vertical velocity
    float verticalVelocity = (this->sensorPacket.altitude - this->lastAltitude) / (this->deltaTime / 1000.0);
    this->lastAltitude = this->sensorPacket.altitude;

    // add vertical velocity to cyclic buffer
    size_t buffLen = ARRAY_SIZE(this->verticalVelocityBuffer);
    Utility::circBufInsert(buffLen, this->verticalVelocityBuffer, &this->bufferIndex, verticalVelocity);
    
    // If the average vertical velocity <= 0 for more than 30 cycles, rocket has passed apogee
    this->apogeePassed = this->apogeeDebouncer.checkOut(Utility::average(buffLen, this->verticalVelocityBuffer) <= 0);
}

//! @details max 8 seconds until deploy
State *Coast::nextState_impl()
{
    // Transition state if condition met
    if (this->apogeePassed)
    {
        return new DrogueDescent(this->sensors, this->stateEstimator);
    }

    // if the state hasn't changed for much more than the expected COAST time, go to abort
    // 1.5 * TIME_IN_COAST == 28.5 seconds
    if (this->currentTime > 1.5 * TIME_IN_COAST)
    {
        return new Abort(this->sensors, this->stateEstimator);
    }
    return nullptr;
}

enum StateId Coast::getId()
{
    return StateId::ID_Coast;
}
