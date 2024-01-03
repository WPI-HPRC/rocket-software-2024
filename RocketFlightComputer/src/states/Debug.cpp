#include "Debug.h"
#include "State.h"

Debug::Debug() {
}

void Debug::initialize_impl() {
	// Initialize sensors
	// We **definitely** don't want to spin forever here, but it doesn't hurt to try multiple times if initializing fails at first
}

void Debug::loop_impl() {
	// Read bno for example
    Serial.println("+=== Barometer ===+");
    Serial.print("Pressure: "); Serial.println(telemPacket.pressure);
    Serial.print("Altitude: "); Serial.println(telemPacket.altitude);

    Serial.println("+=== Accelerometer ===+");
    Serial.print("Accel X: "); Serial.println(telemPacket.accelX);
    Serial.print("Accel Y: "); Serial.println(telemPacket.accelY);
    Serial.print("Accel Z: "); Serial.println(telemPacket.accelZ);

    Serial.println("+=== Gyroscope ===+");
    Serial.print("Gyro X: "); Serial.println(telemPacket.gyroX);
    Serial.print("Gyro Y: "); Serial.println(telemPacket.gyroY);
    Serial.print("Gyro Z: "); Serial.println(telemPacket.gyroZ);

    Serial.println("+=== Magnetometer ===+");
    Serial.print("Mag X: "); Serial.println(telemPacket.magX);
    Serial.print("Mag Y: "); Serial.println(telemPacket.magY);
    Serial.print("Mag Z: "); Serial.println(telemPacket.magZ);
}

//! @details If we are separating this from `Launch`, we need a time limit on this state or something
State *Debug::nextState_impl()
{
	return nullptr;
}
