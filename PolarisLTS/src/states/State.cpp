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

float State::pressureToAltitude(float pressure) {
    // physical parameters for model
    const float pb = 101325;   // [Pa] pressure at sea level
    const float Tb = 288.15;   // [K] temperature at seal level
    const float Lb = -0.0065;  // [K/m] standard temperature lapse rate
    const float hb = 0;        // [m] height at bottom of atmospheric layer (sea level)
    const float R = 8.31432;   // [N*m/mol*K] universal gas constant
    const float g0 = 9.80665;  // [m/s^2] Earth standard gravity
    const float M = 0.0289644; // [kg/mol] molar mass of Earth's air

    // convert pressure from [mBar] to [Pa]
    float pressure_Pa = pressure * 100;

    // compute altitude from formula
    return hb + (Tb / Lb) * (pow((pressure_Pa / pb), (-R * Lb / (g0 * M))) - 1);
};