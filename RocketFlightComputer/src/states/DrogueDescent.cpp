#include "State.h"
#include "DrogueDescent.h"
#include "Sensors.h"

DrogueDescent::DrogueDescent(struct Sensors *sensors, StateEstimator *stateEstimator, FlashChip *flashChip) : State(sensors, stateEstimator, flashChip) {}

DrogueDescent::DrogueDescent()
{
}

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
    if (average < MAIN_DEPLOY_VELOCITY)
    {
        Serial.println("Reached main deploy velocity!");
        drogueDescentRateMatched = true;
    }
}

State *DrogueDescent::nextState_impl()
{
    if (drogueDescentRateMatched)
    {
        return new MainDescent(sensors, stateEstimator, flash);
    }

    // if the state hasn't changed for much more than the expected DROGUE time, go to abort
    // 1.2 * TIME_IN_DROGUE_DESCENT == 85.2 seconds
    if (this->currentTime > 1.2 * TIME_IN_DROGUE_DESCENT)
    {
        return new Abort(sensors, stateEstimator, flash);
    }
}
