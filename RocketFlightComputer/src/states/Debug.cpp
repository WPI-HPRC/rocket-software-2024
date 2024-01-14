#include "Debug.h"
#include "State.h"

Debug::Debug(QuatStateEstimator * ekf) {
    this->ekf = ekf;
}

void Debug::initialize_impl() {
	// Initialize sensors
	// We **definitely** don't want to spin forever here, but it doesn't hurt to try multiple times if initializing fails at first
}

void Debug::loop_impl() {
    this->x_state = ekf->onLoop(sensorPacket);

    // Update Sensor Packet with EKF
    this->sensorPacket.q = x_state(0);
    this->sensorPacket.i = x_state(1);
    this->sensorPacket.j = x_state(2);
    this->sensorPacket.k = x_state(3);

    Serial.println("+=== GPS ===+");
    Serial.print("Latitude: "); Serial.println(sensorPacket.gpsLat, 4);
    Serial.print("Longitude: "); Serial.println(sensorPacket.gpsLong, 4);
    Serial.print("Altitude AGL: "); Serial.println(sensorPacket.gpsAltAGL, 4);
    Serial.print("Altitude MSL: "); Serial.println(sensorPacket.gpsAltMSL, 4);

    Serial.println(this->deltaTime);

	// Read bno for example
    // Serial.println("+=== Barometer ===+");
    // Serial.print("Pressure: "); Serial.println(sensorPacket.pressure);
    // Serial.print("Altitude: "); Serial.println(sensorPacket.altitude);

    // Serial.println("+=== Accelerometer ===+");
    // Serial.print("Accel X: "); Serial.println(sensorPacket.accelX);
    // Serial.print("Accel Y: "); Serial.println(sensorPacket.accelY);
    // Serial.print("Accel Z: "); Serial.println(sensorPacket.accelZ);

    // Serial.println("+=== Gyroscope ===+");
    // Serial.print("Gyro X: "); Serial.println(sensorPacket.gyroX);
    // Serial.print("Gyro Y: "); Serial.println(sensorPacket.gyroY);
    // Serial.print("Gyro Z: "); Serial.println(sensorPacket.gyroZ);

    // Serial.println("+=== Magnetometer ===+");
    // Serial.print("Mag X: "); Serial.println(sensorPacket.magX);
    // Serial.print("Mag Y: "); Serial.println(sensorPacket.magY);
    // Serial.print("Mag Z: "); Serial.println(sensorPacket.magZ);
}

//! @details If we are separating this from `Launch`, we need a time limit on this state or something
State *Debug::nextState_impl()
{
	return nullptr;
}
