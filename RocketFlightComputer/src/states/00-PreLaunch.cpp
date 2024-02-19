#include "00-PreLaunch.h"
#include "State.h"
#include "01-Launch.h"
#include "07-Debug.h"
#include "Sensors.h"

PreLaunch::PreLaunch(struct Sensors *sensors, StateEstimator *stateEstimator) : State(sensors, stateEstimator) {}

float PreLaunch::avgAccelZ()
{
    float sum = 0;
    uint8_t len = sizeof(this->accelReadingBuffer) / sizeof(float);

    for (uint8_t i = 0; i < len; i++)
    {
        sum += this->accelReadingBuffer[i];
    }

    return sum / len;
}

void PreLaunch::initialize_impl()
{
}

void PreLaunch::loop_impl()
{
    if (!sensorPacket.gpsLock)
    {
        Serial.println("[PreLaunch] Gps Lock Failed...");

        // delay(100);
        // return;
    }

    if (this->stateEstimatorInitialized)
    {
        this->accelReadingBuffer[this->buffIdx++] = this->sensorPacket.accelZ;
        this->buffIdx %= sizeof(this->accelReadingBuffer) / sizeof(float);
        return;
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
    if (!this->stateEstimatorInitialized)
    {
        BLA::Matrix<10> x_0 = {1, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        this->stateEstimator = new StateEstimator(x_0, 0.025);
        this->stateEstimatorInitialized = true;
    }

    launched = launchDebouncer.checkOut(this->avgAccelZ() > LAUNCH_ACCEL_THRESHOLD);
}

State *PreLaunch::nextState_impl()
{

#ifdef DEBUG_MODE
    if (sensorPacket.gpsLock)
    {
        return new Debug(this->sensors, this->ekf);
    }
#endif

    if (launched)
    {
        return new Launch(sensors, stateEstimator);
    }

    return nullptr;
}

enum StateId PreLaunch::getId()
{
    return StateId::ID_PreLaunch;
}
