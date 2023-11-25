#include "PreLaunch.h"
#include <states/State.h>

#include <states/Simulation/Simulation.h>

PreLaunch::PreLaunch() {
    this->name = "PreLaunch";
}

void PreLaunch::initialize_impl() {
    // Obtain initial quaternion orientation
    // Eigen::Vector4f initialQuat = calculateInitialOrientation();
    Serial.println("Initializing Prelaunch");

    stateEstimator = new QuatStateEstimator({1,0,0,0}, 0.025);

}

void PreLaunch::loop_impl() {
    // Read bno for example
}

//! @details If we are separating this from `Launch`, we need a time limit on this state or something
State *PreLaunch::nextState_impl() {
    if (this->currentTime > MAX_PRELAUNCH_TIME)
    {
        return new Simulation(*stateEstimator);
        // return new Launch();
    }
    return nullptr;
}

// Eigen::Vector4f PreLaunch::calculateInitialOrientation() {
//     // Convert sensor values to SI Units
//     float accelX = sensorData.ac_x * 9.81;
//     float accelY = sensorData.ac_y * 9.81;
//     float accelZ = sensorData.ac_z * 9.81;

//     float roll = atan2(-accelY, accelZ);
//     float pitch = atan2(accelX, sqrt(accelY*accelY* + accelZ*accelZ));

//     Eigen::Quaternionf initialQuaternion = Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()) *
//                                     Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
//                                     Eigen::AngleAxisf(0.0f, Eigen::Vector3f::UnitZ());

//     Serial.println("Quat");

//     return initialQuaternion.coeffs();

// };