#include "State.h"
#include "Launch.h"
#include "Coast.h"
#include "Sensors.h"

Launch::Launch(struct Sensors *sensors, StateEstimator *stateEstimator, FlashChip *flashChip) : State(sensors, stateEstimator, flashChip) {}

void Launch::initialize_impl() {}

void Launch::loop_impl()
{
    // old motorBurnoutDetect code
    // accel value gets updated in sensor reading fcn
    // add to cyclic buffer
    transitionBufAcc[transitionBufIndAcc] = sensorPacket.accelZ;
    // take running average value
    float sum = 0.0;
    float average = 0.0;
    for (int i = 0; i < 10; i++)
    {
        sum += transitionBufAcc[i];
    }
    average = sum / 10.0;

    transitionBufIndAcc = (transitionBufIndAcc + 1) % 10;
    // compare running average value to defined threshold
    if (average < 0)
    {
        transitionBufIndAcc = 0;
        Serial.println("Motor burnout detected!");
        motorBurnout = true;
        // TODO: debounce motorBurnout with counter for like 50 loops etc
    }
}

State *Launch::nextState_impl()
{
    // Stay in this state for at least 3 seconds to prevent airbrake activation
    if (this->currentTime > MIN_LAUNCH_TIME && motorBurnout)
    {
        return new Coast(sensors, stateEstimator, flash);
    }
    // TODO: add abort
    return nullptr;
}
