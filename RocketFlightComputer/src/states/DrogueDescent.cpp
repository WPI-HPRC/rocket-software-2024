#include "State.h"
#include "DrogueDescent.h"
#include "MainDescent.h"
#include "Abort.h"
#include "Sensors.h"

DrogueDescent::DrogueDescent(struct Sensors *sensors, StateEstimator *stateEstimator) : State(sensors, stateEstimator) {}

void DrogueDescent::initialize_impl() {}

// TODO: Debounce
void DrogueDescent::loop_impl()
{
    // calculate vertical velocity
    float verticalVelocity = (sensorPacket.altitude - lastAltitude) / (deltaTime / 1000.0);
    lastAltitude = sensorPacket.altitude;

    // Velocity value gets updated in sensor reading fcn
    // add to cyclic buffer
    transitionBuffVerticalVelocity[transitionBuffIndexVerticalVelocity] = verticalVelocity;
    // take running average value
    float sum = 0.0;
    float average = 0.0;
    for (int i = 0; i < 10; i++)
    {
        sum += transitionBuffVerticalVelocity[i];
    }
    average = sum / 10.0;

    transitionBuffIndexVerticalVelocity = (transitionBuffIndexVerticalVelocity + 1) % 10;

    // if the average vertical velocity is less that the expected velocity at main deploy, swap states
    drogueDescentRateMatched = drogueDescentDebouncer.checkOut(average <= MAIN_DEPLOY_VELOCITY);
}

State *DrogueDescent::nextState_impl()
{
    if (drogueDescentRateMatched)
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
