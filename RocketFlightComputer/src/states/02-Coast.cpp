
// Coast state
// This state occurs after the motor has burned out as the rocket is on its way to apogee

#include "02-Coast.h"
#include "03-DrogueDescent.h"
#include "06-Abort.h"
#include "FlightParams.hpp"
#include "State.h"

Coast::Coast(struct Sensors *sensors, AttitudeStateEstimator *attitudeStateEstimator) : State(sensors, attitudeStateEstimator) {}

Coast::~Coast() {
#ifndef NO_SERVO
    airbrakesServo.write(AIRBRAKE_RETRACTED);
#endif
}

void Coast::initialize_impl() {}

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

    #ifdef SERVO_TEST
    Serial.printf("Servo position: %d\n", this->telemPacket.servoPosition);
    #endif

    // Handle airbrake control
    // TODO: I don't really know which values correspond to which positions yet, so these values are subject to change
    // TODO (maybe) mock servo for fun, wouldn't be all that useful tho
#ifndef NO_SERVO
    switch (this->servoState) {
    case WAIT:
        airbrakesServo.write(AIRBRAKE_RETRACTED);
        if (this->currentTime - this->lastTransitionTime >= AIRBRAKE_WAIT_AFTER_TRANSITION) {
            this->lastTransitionTime = this->currentTime;
            this->servoState = FIRST_STEP;
        }
        break;
    case FIRST_STEP:
        airbrakesServo.write(AIRBRAKE_FIRST_EXTENSION);
        if (this->currentTime - this->lastTransitionTime >= AIRBRAKE_FIRST_EXTENSION_TIME) {
            this->lastTransitionTime = this->currentTime;
            this->servoState = SECOND_STEP;
        }
        break;
    case SECOND_STEP:
        airbrakesServo.write(AIRBRAKE_SECOND_EXTENSION);
        if (this->currentTime - this->lastTransitionTime >= AIRBRAKE_SECOND_EXTENSION_TIME) {
            this->lastTransitionTime = this->currentTime;
            this->servoState = DONE;
        }
        break;
    case DONE:
        airbrakesServo.write(AIRBRAKE_RETRACTED);
        break;
    }
#endif
}

//! @details max 8 seconds until deploy
State *Coast::nextState_impl()
{
    // Transition state if condition met
    if (apogeePassed)
    {
        return new DrogueDescent(sensors, attitudeStateEstimator);
    }

    // if the state hasn't changed for much more than the expected COAST time, go to abort
    // 1.5 * TIME_IN_COAST == 28.5 seconds
    if (this->currentTime > 1.5 * TIME_IN_COAST)
    {
        return new Abort(sensors, attitudeStateEstimator);
    }
    return nullptr;
}

enum StateId Coast::getId()
{
    return StateId::ID_Coast;
}
