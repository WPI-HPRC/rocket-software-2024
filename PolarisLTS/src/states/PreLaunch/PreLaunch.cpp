#include "PreLaunch.h"
#include <states/State.h>

#include <states/Simulation/Simulation.h>


PreLaunch::PreLaunch()
{
    this->name = "PreLaunch";
}
void PreLaunch::initialize_impl()
{
    // Initialize sensors
    // We **definitely** don't want to spin forever here, but it doesn't hurt to try multiple times if initializing fails at first

}
void PreLaunch::loop_impl()
{
    // Read bno for example
}
//! @details If we are separating this from `Launch`, we need a time limit on this state or something
State *PreLaunch::nextState_impl()
{
    if (this->currentTime > MAX_PRELAUNCH_TIME)
    {
        return new Simulation();
        // return new Launch();
    }
    return nullptr;
}