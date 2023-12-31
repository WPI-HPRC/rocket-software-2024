#include "PreLaunch.h"
#include <states/State.h>

#include <states/Simulation/Simulation.h>

PreLaunch::PreLaunch() {
    this->name = "PreLaunch";
}

void PreLaunch::initialize_impl() {
    // Obtain initial quaternion orientation
    Serial.println("Initializing Prelaunch");

    Eigen::Vector<float, 10> x_0 = {1,0,0,0,0,0,0,0,0,0};
    // BLA::Matrix<4> x_0 = {1,0,0,0};
    ekf = new StateEstimator(x_0, 0.01);
}

void PreLaunch::loop_impl() {

    Eigen::Vector<float, 10> x_0 = {1,0,0,0,0,0,0,0,0,0};
    
    telemPacket.accelX = sensorData.ac_x;
    telemPacket.accelY = sensorData.ac_y;
    telemPacket.accelZ = sensorData.ac_z;

    telemPacket.gyroX = sensorData.gy_x;
    telemPacket.gyroY = sensorData.gy_y;
    telemPacket.gyroZ = sensorData.gy_z;

    telemPacket.magX = sensorData.mag_x;
    telemPacket.magY = sensorData.mag_y;
    telemPacket.magZ = sensorData.mag_z;

    telemPacket.pressure = sensorData.Pressure;
    telemPacket.altitude = pressureToAltitude(sensorData.Pressure);
    telemPacket.timestamp = millis();
    telemPacket.q = x_0(0);
    telemPacket.i = x_0(1);
    telemPacket.j = x_0(2);
    telemPacket.k = x_0(3);
}

//! @details If we are separating this from `Launch`, we need a time limit on this state or something
State *PreLaunch::nextState_impl() {
    if (this->currentTime > MAX_PRELAUNCH_TIME)
    {
        return new Simulation(this->ekf);
        // return new Launch();
    }
    return nullptr;
}
// BLA::Matrix<4> PreLaunch::calculateInitialOrientation() {
//     // Convert sensor values to SI Units
//     float accelX = sensorData.ac_x * 9.81;
//     float accelY = sensorData.ac_y * 9.81;
//     float accelZ = sensorData.ac_z * 9.81;

//     float roll = atan2(-accelY, accelZ);
//     float pitch = atan2(accelX, sqrt(accelY*accelY* + accelZ*accelZ));
//     float yaw = 0;

//     BLA::Matrix<4> orientation;
//     orientation(0, 0) = cos(roll) * cos(pitch) * cos(yaw) - sin(roll) * sin(yaw);
//     orientation(0, 1) = -cos(roll) * cos(pitch) * sin(yaw) - sin(roll) * cos(yaw);
//     orientation(0, 2) = cos(roll) * sin(pitch);
//     orientation(0, 3) = 0;

//     orientation(1, 0) = sin(roll) * cos(pitch) * cos(yaw) + cos(roll) * sin(yaw);
//     orientation(1, 1) = -sin(roll) * cos(pitch) * sin(yaw) + cos(roll) * cos(yaw);
//     orientation(1, 2) = sin(roll) * sin(pitch);
//     orientation(1, 3) = 0;

//     orientation(2, 0) = -sin(pitch) * cos(yaw);
//     orientation(2, 1) = sin(pitch) * sin(yaw);
//     orientation(2, 2) = cos(pitch);
//     orientation(2, 3) = 0;

//     orientation(3, 0) = 0;
//     orientation(3, 1) = 0;
//     orientation(3, 2) = 0;
//     orientation(3, 3) = 1;

//     return orientation;

// };