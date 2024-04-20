#include "00-PreLaunch.h"
#include "State.h"
#include "01-Launch.h"
#include "utility.hpp"

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
    if (!telemPacket.gpsLock)
    {
        // Serial.println("[PreLaunch] Gps Lock Failed...");

        // delay(100);
        // return;
    }
#ifndef NO_SDCARD
    if (!sdCardInitialized) {
        if (sd.begin(9)) {
            int fileIdx = 0;
            for (int i = 0; i < 50; i++) {
                char filename[100];
                sprintf(filename, "flightData%d.bin", fileIdx++);
                Serial.print("Trying file: ");
                Serial.println(filename);
                if (!sd.exists(filename)) {
                    dataFile = sd.open(filename, FILE_WRITE);
                    break;
                }
            }
            sdCardInitialized = true;
        }
    }
#endif
    Serial.print("ACCEL: ");
    Serial.print(telemPacket.accelX);
    Serial.print(", ");
    Serial.print(telemPacket.accelY);
    Serial.print(", ");
    Serial.println(telemPacket.accelZ);

    if (this->stateEstimator->initialized)
    {
        this->accelReadingBuffer[this->buffIdx++] = this->telemPacket.accelZ;
        this->buffIdx %= sizeof(this->accelReadingBuffer) / sizeof(float);
        launched = launchDebouncer.checkOut(this->avgAccelZ() > LAUNCH_ACCEL_THRESHOLD);
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
    if (!this->stateEstimator->initialized)
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

        BLA::Matrix<3> crossProd1 = Utility::crossProduct(a, m);
        BLA::Matrix<3> crossProd2 = Utility::crossProduct(a,crossProd1); 

        BLA::Matrix<4> q_0 = {
            0.5 * sqrt(1 + a(0) + crossProd1(1) + crossProd2(2)),
            0.5 * std::copysign(sqrt(1 + a(0) - crossProd1(1) - crossProd2(2)), crossProd2(1) - crossProd1(2)),
            0.5 * std::copysign(sqrt(1 - a(0) + crossProd1(1) - crossProd2(2)), crossProd1(2) - crossProd2(1)),
            0.5 * std::copysign(sqrt(1 - a(0) - crossProd1(1) + crossProd2(2)), crossProd2(0) - crossProd1(1) + crossProd1(0) - crossProd2(0))
        };

        Serial.println("<----- Initial Quaternion ----->");
        for (int i = 0; i < q_0.Rows; i++) {
            for (int j = 0; j < q_0.Cols; j++) {
                Serial.print(String(q_0(i,j)) + "\t");
            }
            Serial.println("");
        };

        BLA::Matrix<10> x_0 = {q_0(0), q_0(1), q_0(2), q_0(3), 0, 0, 0, 0, 0, 0};
        this->stateEstimator->init(x_0, 0.025);
        
        Serial.println("[Prelaunch] Initialized EKF");
    }
}

State *PreLaunch::nextState_impl()
{
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
