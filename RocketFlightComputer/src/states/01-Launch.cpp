#include "State.h"
#include "01-Launch.h"
#include "02-Coast.h"
#include "06-Abort.h"
#include "Sensors.h"

Launch::Launch(struct Sensors *sensors, StateEstimator *stateEstimator) : State(sensors, stateEstimator) {}

void Launch::initialize_impl() {}

void Launch::loop_impl()
{
    // detect motor burnout via acceleration
    // add current Z acceleration to cyclic buffer
    accelerationBuffer[bufferIndex] = sensorPacket.accelZ;

    // take the average of the buffer
    float sum = 0.0;
    float averageZAccel = 0.0;
    for (int i = 0; i < 10; i++)
    {
        sum += accelerationBuffer[i];
    }
    averageZAccel = sum / 10.0;

    bufferIndex = (bufferIndex + 1) % 10;

    // if the average Z acceleration is less than the burnout threshold for 30 cycles, motor has burned out
    motorBurnout = motorBurnoutDebouncer.checkOut(averageZAccel < BURNOUT_THRESHOLD);
}

State *Launch::nextState_impl()
{
    // Stay in this state for at least 3 seconds to prevent airbrake activation under motor power
    if (this->currentTime >= MOTOR_BURN_TIME && motorBurnout)
    {
        return new Coast(sensors, stateEstimator);
    }

    // if state hasn't changed for much more than 2x motor burnout time, go to abort
    if (this->currentTime > 2 * MOTOR_BURN_TIME)
    {
        return new Abort(sensors, stateEstimator);
    }

    return nullptr;
}

enum StateId Launch::getId()
{
    return StateId::ID_Launch;
}
