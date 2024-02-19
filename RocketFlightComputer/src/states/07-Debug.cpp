#include "07-Debug.h"
#include "State.h"

Debug::Debug(struct Sensors *sensors, StateEstimator * ekf) : State(sensors, ekf) {}

void Debug::initialize_impl() {
	// Initialize sensors
	// We **definitely** don't want to spin forever here, but it doesn't hurt to try multiple times if initializing fails at first
    // Serial.println("LIN_ACCELX,LIN_ACCELY,LIN_ACCELZ,W,I,J,K,LAT,LON,ALT_MSL_GPS,ALT_BARO,BARO,ACCELX,ACCELY,ACCELZ,GYROX,GYROY,GYROZ,MAGX,MAGY,MAGZ,Time");
}

void Debug::loop_impl() {
    // Update Sensor Packet with EKF
    this->telemPacket.w = x_state(0);
    this->telemPacket.i = x_state(1);
    this->telemPacket.j = x_state(2);
    this->telemPacket.k = x_state(3);  

    // Serial.print("QUAT|");
    // Serial.print(telemPacket.w); Serial.print(",");
    // Serial.print(telemPacket.i); Serial.print(",");
    // Serial.print(telemPacket.j); Serial.print(",");
    // Serial.println(telemPacket.k);
    
    // this->sensorPacket.X = x_state(4);
    // this->sensorPacket.Y = x_state(5);
    // this->sensorPacket.Z = x_state(6);
    // Serial.print(this->sensorPacket.w); Serial.print(",");
    // Serial.print(this->sensorPacket.i); Serial.print(",");
    // Serial.print(this->sensorPacket.j); Serial.print(",");
    // Serial.print(this->sensorPacket.k); Serial.print(",");
    // Serial.print(this->sensorPacket.gpsLat,4); Serial.print(",");
    // Serial.print(this->sensorPacket.gpsLong,4); Serial.print(",");
    // Serial.print(this->sensorPacket.gpsAltMSL); Serial.print(",");
    // Serial.print(this->sensorPacket.altitude); Serial.print(",");
    // Serial.print(this->sensorPacket.pressure); Serial.print(",");
    // Serial.print(this->sensorPacket.accelX); Serial.print(",");
    // Serial.print(this->sensorPacket.accelY); Serial.print(",");
    // Serial.print(this->sensorPacket.accelZ); Serial.print(",");
    // Serial.print(this->sensorPacket.gyroX); Serial.print(",");
    // Serial.print(this->sensorPacket.gyroY); Serial.print(",");
    // Serial.print(this->sensorPacket.gyroZ); Serial.print(",");
    // Serial.print(this->sensorPacket.magX); Serial.print(",");
    // Serial.print(this->sensorPacket.magY); Serial.print(",");
    // Serial.print(this->sensorPacket.magZ); Serial.print(",");
    // Serial.println(this->sensorPacket.epochTime);

    // Serial.println(this->deltaTime);

    // Serial.println("+=== GPS ===+");
    // Serial.print("Latitude: "); Serial.println(sensorPacket.gpsLat, 4);
    // Serial.print("Longitude: "); Serial.println(sensorPacket.gpsLong, 4);
    // Serial.print("Altitude AGL: "); Serial.println(sensorPacket.gpsAltAGL, 4);
    // Serial.print("Altitude MSL: "); Serial.println(sensorPacket.gpsAltMSL, 4);

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

enum StateId Debug::getId() {
	return StateId::ID_Debug;
}
