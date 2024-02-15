#include "MainDescent.h"
#include "Abort.h"
#include "Recovery.h"
#include "State.h"
#include "Sensors.h"

MainDescent::MainDescent(struct Sensors *sensors, StateEstimator *stateEstimator) : State(sensors, stateEstimator) {}

void MainDescent::initialize_impl() {}

void MainDescent::loop_impl()
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
    // if average vertical velocity is below the expected landing velocity, landed
    // TODO: debounce landed
    if (average < LANDING_VELOCITY)
    {
        Serial.println("Landed!");
        landed = true;
    }
}

State *MainDescent::nextState_impl()
{
    if (landed)
    {
        return new Recovery(sensors, stateEstimator);
    }

    // if the state hasn't changed for much more than the expected MAIN_DESCENT time, go to abort
    // 1.1 * TIME_IN_MAIN_DESCENT == 96.8 seconds
    if (this->currentTime > 1.1 * TIME_IN_MAIN_DESCENT)
    {
        return new Abort(sensors, stateEstimator);
    }

    return nullptr;
}

enum StateId MainDescent::getId()
{
    return StateId::ID_MainDescent;
}
