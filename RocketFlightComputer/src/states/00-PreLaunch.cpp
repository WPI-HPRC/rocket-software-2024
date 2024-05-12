#include "00-PreLaunch.h"
#include "State.h"
#include "01-Launch.h"
#include "utility.hpp"

PreLaunch::PreLaunch(struct Sensors *sensors, AttitudeStateEstimator *attitudeStateEstimator, KinematicStateEstimator *kinematicStateEstimator) : State(sensors, attitudeStateEstimator, kinematicStateEstimator) {}

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
    if (!telemPacket.gpsLock)
    {
        // Serial.println("[PreLaunch] Gps Lock Failed...");

        // delay(100);
        // return;
    }
    if (this->attitudeStateEstimator->initialized && this->kinematicStateEstimator->initialized)
    {
        this->accelReadingBuffer[this->buffIdx++] = this->telemPacket.accelZ;
        this->buffIdx %= sizeof(this->accelReadingBuffer) / sizeof(float);
        launched = launchDebouncer.checkOut(this->avgAccelZ() > LAUNCH_ACCEL_THRESHOLD);
        return;
    }

    // Intialize EKF
    if (!this->attitudeStateEstimator->initialized)
    {
        // Calculate Initial Quaternion using Accel and Mag
        // Normalize Acceleration Vector
        BLA::Matrix<3> a = {telemPacket.accelX, telemPacket.accelY, telemPacket.accelZ};
        float aLen = BLA::Norm(a);
        if (aLen != 0) {
            a /= aLen;
        }

        // Normalize Magnetometer Vector
        BLA::Matrix<3> m = {telemPacket.magX, telemPacket.magY, telemPacket.magZ};
        float mLen = BLA::Norm(m);
        if (mLen != 0) {
            m /= mLen;
        }

        // Observation Matrix

        BLA::Matrix<3> crossProd1 = Utility::crossProduct(a,m);
        BLA::Matrix<3> crossProd2 = Utility::crossProduct(crossProd1, a);
        
        BLA::Matrix<3,3> C = {
            crossProd2(0), crossProd1(0), a(0),
            crossProd2(1), crossProd1(1), a(1),
            crossProd2(3), crossProd1(2), a(2)
        };

        BLA::Matrix<4> q_0 = {
            0.5f * sqrt(C(1,1) + C(2,2) + C(3,3) + 1),
            0.5f * std::copysign(1, C(2,1) - C(1,2)) * sqrt(C(0,0) - C(1,1) - C(2,2) + 1),
            0.5f * std::copysign(1, C(0,2) - C(2,0)) * sqrt(C(1,1) - C(2,2) - C(0,0) + 1),
            0.5f * std::copysign(1, C(1,0) - C(0,1)) * sqrt(C(2,2) - C(0,0) - C(1,1) + 1)
        };

        Serial.println("<----- Initial Quaternion ----->");
        for (int i = 0; i < q_0.Rows; i++) {
            for (int j = 0; j < q_0.Cols; j++) {
                Serial.print(String(q_0(i,j)) + "\t");
            }
            Serial.println("");
        };

        this->attitudeStateEstimator->init(q_0, 0.025);
        
        Serial.println("[Prelaunch] Initialized Attitude EKF");
    }

    if (!this->kinematicStateEstimator->initialized) {
        // float r_adj = Utility::r_earth + sensorPacket.gpsAltMSL; // [m]
        // float N_earth = Utility::a_earth / sqrt(1 - pow(Utility::e_earth, 2) * pow(sin(sensorPacket.gpsLat), 2));

        // float X_0 = (N_earth + sensorPacket.gpsAltAGL) * cos(sensorPacket.gpsLat * DEG_TO_RAD) * cos(sensorPacket.gpsLong * DEG_TO_RAD);
        // float Y_0 = (N_earth + sensorPacket.gpsAltAGL) * cos(sensorPacket.gpsLat * DEG_TO_RAD) * sin(sensorPacket.gpsLong * DEG_TO_RAD);
        // float Z_0 = (((Utility::b_earth * Utility::b_earth) / (Utility::a_earth * Utility::a_earth)) * N_earth + sensorPacket.gpsAltAGL) * sin(sensorPacket.gpsLat * DEG_TO_RAD);
        // float Z_0 = (N_earth*(1-pow(Utility::e_earth,2))+sensorPacket.gpsAltAGL)*sin(sensorPacket.gpsLat);
        BLA::Matrix<6> x_0 = {0,0,0, 0,0,0};
        this->kinematicStateEstimator->init(x_0, telemPacket.altitude, 0.025);

        Serial.println("[Prelaunch] Initialized Kinematic EKF");
    }
}

State *PreLaunch::nextState_impl()
{
    if (launched)
    {
        return new Launch(sensors, attitudeStateEstimator, kinematicStateEstimator);
    }

    return nullptr;
}

enum StateId PreLaunch::getId()
{
    return StateId::ID_PreLaunch;
}
