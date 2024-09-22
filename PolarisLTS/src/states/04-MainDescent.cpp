#include "04-MainDescent.h"
#include "06-Abort.h"
#include "05-Recovery.h"
#include "State.h"

MainDescent::MainDescent(Sensorboard *sensors, AttitudeStateEstimator *attitudeStateEstimator) : State(sensors, attitudeStateEstimator) {}

void MainDescent::initialize_impl() {}

void MainDescent::loop_impl()
{
    // calculate vertical velocity
    float verticalVelocity = (telemPacket.altitude - lastAltitude) / (deltaTime / 1000.0);
    lastAltitude = telemPacket.altitude;

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
    landed = landedDebouncer.checkOut(abs(averageVerticalVelocity) < LANDING_VELOCITY);
}

State *MainDescent::nextState_impl()
{
    // Transition state if condition met
    if (landed)
    {
        return new Recovery(sensors, attitudeStateEstimator);
    }

    // if the state hasn't changed for much more than the expected MAIN_DESCENT time, go to abort
    // 1.1 * TIME_IN_MAIN_DESCENT == 96.8 seconds
    if (this->currentTime > 1.1 * TIME_IN_MAIN_DESCENT)
    {
        return new Abort(sensors, attitudeStateEstimator);
    }

    return nullptr;
}

enum StateId MainDescent::getId()
{
    return StateId::ID_MainDescent;
}
