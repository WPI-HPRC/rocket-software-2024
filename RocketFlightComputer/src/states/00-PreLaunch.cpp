#include "00-PreLaunch.h"
#include "State.h"
#include "01-Launch.h"
#include "Sensors.h"
#include "utility.hpp"

PreLaunch::PreLaunch(struct Sensors *sensors, StateEstimator *stateEstimator) : State(sensors, stateEstimator) {}

void PreLaunch::initialize_impl()
{
}

void PreLaunch::loop_impl()
{
    if (!this->sensorPacket.gpsLock)
    {
        Serial.println("[PreLaunch] Gps Lock Failed...");

        // delay(100);
        // return;
    }

    // float r_adj = Utility::r_earth + sensorPacket.gpsAltMSL; // [m]
    // float N_earth = Utility::a_earth / sqrt(1 - pow(Utility::e_earth, 2) * pow(sin(sensorPacket.gpsLat), 2));

    // float X_0 = (N_earth + sensorPacket.gpsAltAGL) * cos(sensorPacket.gpsLat * DEG_TO_RAD) * cos(sensorPacket.gpsLong * DEG_TO_RAD);
    // float Y_0 = (N_earth + sensorPacket.gpsAltAGL) * cos(sensorPacket.gpsLat * DEG_TO_RAD) * sin(sensorPacket.gpsLong * DEG_TO_RAD);
    // float Z_0 = (((Utility::b_earth * Utility::b_earth) / (Utility::a_earth * Utility::a_earth)) * N_earth + sensorPacket.gpsAltAGL) * sin(sensorPacket.gpsLat * DEG_TO_RAD);
    // float Z_0 = (N_earth*(1-pow(Utility::e_earth,2))+sensorPacket.gpsAltAGL)*sin(sensorPacket.gpsLat);

    // Serial.println("[Pre-Launch] Initial GPS Position Acquired!");
    // Serial.print("Latitude: ");
    // Serial.println(sensorPacket.gpsLat, 4);
    // Serial.print("Longitude: ");
    // Serial.println(sensorPacket.gpsLong, 4);
    // Serial.print("Initial Position: <");
    // Serial.print(X_0, 4);
    // Serial.print(", ");
    // Serial.print(Y_0, 4);
    // Serial.print(", ");
    // Serial.print(Z_0, 4);
    // Serial.println(">");

    // Intialize EKF
    if (this->stateEstimatorInitialized) {
        size_t buffLen = ARRAY_SIZE(this->accelerationBuffer);
        Utility::circBufInsert(buffLen, this->accelerationBuffer, &this->bufferIndex, this->sensorPacket.accelZ);
        this->launched = this->launchDebouncer.checkOut(Utility::average(buffLen, this->accelerationBuffer) > LAUNCH_ACCEL_THRESHOLD);
    } else {
        BLA::Matrix<10> x_0 = {1, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        this->stateEstimator = new StateEstimator(x_0, 0.025);
        this->stateEstimatorInitialized = true;
    }
}

State *PreLaunch::nextState_impl()
{

#if DEBUG_MODE
    if (this->sensorPacket.gpsLock)
    {
        return new Debug(this->sensors, this->stateEstimator);
    }
#endif

    if (this->launched)
    {
        return new Launch(this->sensors, this->stateEstimator);
    }

    return nullptr;
}

enum StateId PreLaunch::getId()
{
    return StateId::ID_PreLaunch;
}
