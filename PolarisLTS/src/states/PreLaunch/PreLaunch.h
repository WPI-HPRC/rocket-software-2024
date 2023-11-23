#pragma once
#include <states/State.h>
#include <Arduino.h>

#define MAX_PRELAUNCH_TIME 3000
class PreLaunch : public State
{
    _STATE_CLASS_IMPLS_
public:
    PreLaunch();
};