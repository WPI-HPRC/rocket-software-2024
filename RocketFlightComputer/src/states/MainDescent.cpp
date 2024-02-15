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

    // add vertical velocity to cyclic buffer
    verticalVelocityBuffer[bufferIndex] = verticalVelocity;
    
    // average all values in the buffer
    float sum = 0.0;
    float averageVerticalVelocity = 0.0;
    for (int i = 0; i < 10; i++)
    {
        sum += verticalVelocityBuffer[i];
    }
    averageVerticalVelocity = sum / 10.0;

    bufferIndex = (bufferIndex + 1) % 10;

    // if the average vertical velocity is less than the expected landing velocity for 30 cycles, the rocket has landed
    landed = landedDebouncer.checkOut(averageVerticalVelocity < LANDING_VELOCITY);
}

State *MainDescent::nextState_impl()
{
    // Transition state if condition met
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
