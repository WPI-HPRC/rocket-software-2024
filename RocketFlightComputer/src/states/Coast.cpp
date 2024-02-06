#include "Coast.h"

#include "DrogueDescent.h"
#include "Abort.h" <- needed as failsafe
#include "State.h"

Coast::Coast() {}

void Coast::initialize_impl() {}

void Coast::loop_impl() {
    // Velocity value gets updated in sensor reading fcn
    // add to cyclic buffer
    transitionBufVelVert[transitionBufIndVelVert] = stateStruct.vel_vert;
    // take running average value
    float sum = 0.0;
    for (int i = 0; i < 10; i++) {
        sum += transitionBufVelVert[i];
    }
    sum = sum / 10.0;

    transitionBufIndVelVert = (transitionBufIndVelVert + 1) % 10;
    // if average vertical velocity is negative, passed apogee
    if (sum < 0) {
        for (int j = 0; j < 10; j++) {
            transitionBufVelVert[j] = 0;
        }
        transitionBufIndVelVert = 0;
        // Serial.println("Apogee detected!");
        apogeePassed = true
    }
}

//! @details max 8 seconds until deploy
State *Coast::nextState_impl() {
    if (this->acceleration > 10) {
        return new Abort();
    }
    if (this->currentTime > MAX_COAST_TIME || apogeePassed) {
        return new DrogueDescent();
    }
    return nullptr;
}
