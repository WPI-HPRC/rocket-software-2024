#include "Coast.h"
#include "State.h"
#include "Sensors.h"

Coast::Coast(struct Sensors *sensors, StateEstimator *stateEstimator, FlashChip *flashChip) : State(sensors, stateEstimator, flashChip) {}

void Coast::initialize_impl() {}

void Coast::loop_impl()
{
    // calculate vertical velocity
    float verticalVelocity = (sensorPacket.altitude - lastAltitude) / (deltaTime / 1000.0);
    lastAltitude = sensorPacket.altitude;

    // Velocity value gets updated in sensor reading fcn
    // add to cyclic buffer
    transitionBufVelVert[transitionBufIndVelVert] = verticalVelocity;
    // take running average value
    float sum = 0.0;
    float average = 0.0;
    for (int i = 0; i < 10; i++)
    {
        sum += transitionBufVelVert[i];
    }
    average = sum / 10.0;

    transitionBufIndVelVert = (transitionBufIndVelVert + 1) % 10;
    // if average vertical velocity is negative, passed apogee
    // TODO Check for acceleration as well
    if (average < 0)
    {
        // TODO: debounce apogeePassed
        Serial.println("Apogee detected!");
        apogeePassed = true;
    }
}

//! @details max 8 seconds until deploy
State *Coast::nextState_impl()
{
    if (this->currentTime > MAX_COAST_TIME || apogeePassed)
    {
        return new DrogueDescent(sensors, stateEstimator, flash);
    }
    return nullptr;
}
