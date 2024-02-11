#include "State.h"
#include "DrogueDescent.h"
#include "Sensors.h"

DrogueDescent::DrogueDescent(struct Sensors *sensors, StateEstimator *stateEstimator, FlashChip *flashChip) : State(sensors, stateEstimator, flashChip) {}

DrogueDescent::DrogueDescent()
{
}

void DrogueDescent::initialize_impl() {}

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

    if (average < DROGUE_DESCENT_RATE)
    {
        Serial.println("Drogue descent rate matched!");
        drogueDescentRateMatched = true;
    }
}

State *DrogueDescent::nextState_impl()
{
    if (drogueDescentRateMatched)
    {
        return new MainDescent(sensors, stateEstimator, flash);
    }

    // if abort (was a 10 second timeout)
    return nullptr;
}
