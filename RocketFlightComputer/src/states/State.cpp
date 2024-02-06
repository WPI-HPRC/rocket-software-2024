#include "State.h"
#include <Arduino.h>

State::State(struct Sensors *sensors, StateEstimator *stateEstimator, FlashChip *flashChip) : sensors(sensors), stateEstimator(stateEstimator), flashChip(flashChip) {}

void State::initialize()
{
    this->startTime = millis();
    initialize_impl();
}

void State::loop()
{
    long long now = millis();
    this->currentTime = now - this->startTime;
    this->deltaTime = now - this->lastLoopTime;
    this->loopCount++;
    this->sensorPacket = this->sensors->readSensors();
    Utility::logData(this->flash, this->sensorPacket);
    this->x_state = this->stateEstimator->onLoop();
    loop_impl();
    this->lastLoopTime = millis();
}

State *State::nextState()
{
    return nextState_impl();
}
