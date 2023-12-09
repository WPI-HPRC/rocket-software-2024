#include "State.h"

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
    loop_impl();
    this->lastLoopTime = millis();
}
State *State::nextState()
{
    return nextState_impl();
}

