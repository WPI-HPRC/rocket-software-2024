#include "Coast.h"
#include "DrogueDescent.h"
#include "Abort.h"
#include "State.h"
#include "Sensors.h"

Coast::Coast(struct Sensors *sensors, StateEstimator *stateEstimator) : State(sensors, stateEstimator) {}

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
    if (apogeePassed)
    {
        return new DrogueDescent(sensors, stateEstimator);
    }
    
    // if the state hasn't changed for much more than the expected COAST time, go to abort
    // 1.5 * TIME_IN_COAST == 28.5 seconds
    if (this->currentTime > 1.5 * TIME_IN_COAST)
    {
        return new Abort(sensors, stateEstimator);
    }
    return nullptr;
}

enum StateId Coast::getId() {
    return StateId::ID_Coast;
}
