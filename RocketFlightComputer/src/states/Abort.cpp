#include "Abort.h"
#include "State.h"

Abort::Abort(FlashChip &flash, Utility::SensorPacket &sensorPacket, String &structString)
{
    this->flash = flash;
    this->sensorPacket = sensorPacket;
    this->structString = structString;
}

void Abort::initialize_impl() {}

void Abort::loop_impl()
{}

State *Abort::nextState_impl()
{
    return nullptr;
}
