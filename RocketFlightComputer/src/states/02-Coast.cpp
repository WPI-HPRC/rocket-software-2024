
// Coast state
// This state occurs after the motor has burned out as the rocket is on its way to apogee

#include "02-Coast.h"
#include "03-DrogueDescent.h"
#include "06-Abort.h"
#include "FlightParams.hpp"
#include "State.h"

Coast::Coast(struct Sensors *sensors, StateEstimator *stateEstimator) : State(sensors, stateEstimator) {}

Coast::~Coast() {
    // I hope this drops causes the servo to lose its PWM signal, so we can configure it to retract in firmware
    // TODO: HOWEVER, the servo is configured to go the the neutral position right now, which might not be retracted
    // If we can't set a specific position on PWM loss in firmware, we need to set it to hold, and then set the position in this state,
    // and NOT detatch until it reaches the target position
    this->airbrakesServo.detach();
}

void Coast::initialize_impl() {
    this->airbrakesServo.attach(25);
}

void Coast::loop_impl()
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
    
    // If the average vertical velocity <= 0 for more than 30 cycles, rocket has passed apogee
    apogeePassed = apogeeDebouncer.checkOut(averageVerticalVelocity <= 0);

    // Handle airbrake control
    // TODO: I don't really know which values correspond to which positions yet, so these values are subject to change
    switch (this->servoState) {
    case WAIT:
        if (this->currentTime >= 1000) {
            this->servoState = EXTEND_FULL;
        }
        break;
    case EXTEND_FULL:
        this->airbrakesServo.write(180);
        if (this->currentTime >= 4000) {
            this->servoState = EXTEND_HALF;
        }
        break;
    case EXTEND_HALF:
        this->airbrakesServo.write(0);
        if (this->currentTime >= 6000) {
            this->servoState = SWEEP_FORWARD;
        }
        break;
    case SWEEP_FORWARD:
        if (this->airbrakesServo.read() >= 180) {
            this->servoState = SWEEP_BACKWARD;
        }
        if (this->loopCount % COAST_AIRBRAKE_INCREMENT_LOOPS == 0) {
            this->airbrakesServo.write(this->airbrakesServo.read() + COAST_AIRBRAKE_INCREMENT_ANGLE);
        }
        break;
    case SWEEP_BACKWARD:
        if (this->airbrakesServo.read() <= 0) {
            this->servoState = SWEEP_FORWARD;
        }
        if (this->loopCount % COAST_AIRBRAKE_INCREMENT_LOOPS == 0) {
            this->airbrakesServo.write(this->airbrakesServo.read() - COAST_AIRBRAKE_INCREMENT_ANGLE);
        }
        break;
    }
}

//! @details max 8 seconds until deploy
State *Coast::nextState_impl()
{
    // Transition state if condition met
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

enum StateId Coast::getId()
{
    return StateId::ID_Coast;
}
