#include "State.h"
#include <Arduino.h>

State::State(struct Sensors *sensors) : sensors(sensors) {}

void State::initialize() {
	this->startTime = millis();
	initialize_impl();
}

void State::loop() {
	long long now = millis();
	this->currentTime = now - this->startTime;
	this->deltaTime = now - this->lastLoopTime;
	this->loopCount++;
	this->sensorPacket = this->sensors->readSensors();
	loop_impl();
	this->lastLoopTime = millis();
	
	/**
	 * Assemble Telemetry packet from sensor packet, this is stuff we want done every loop
	*/
	this->telemPacket.accelX = this->sensorPacket.accelX;
	this->telemPacket.accelY = this->sensorPacket.accelY;
	this->telemPacket.accelZ = this->sensorPacket.accelZ;

	this->telemPacket.gyroX = this->sensorPacket.gyroX;
	this->telemPacket.gyroY = this->sensorPacket.gyroY;
	this->telemPacket.gyroZ = this->sensorPacket.gyroZ;

	this->telemPacket.magX = this->sensorPacket.magX;
	this->telemPacket.magY = this->sensorPacket.magY;
	this->telemPacket.magZ = this->sensorPacket.magZ;

	this->telemPacket.pressure = this->sensorPacket.pressure;

	this->telemPacket.altitude = this->sensorPacket.altitude;

	this->telemPacket.gpsLat = this->sensorPacket.gpsLat;
	this->telemPacket.gpsLong = this->sensorPacket.gpsLong;

	this->telemPacket.epochTime = this->sensorPacket.epochTime;
	this->telemPacket.satellites = this->sensorPacket.satellites;
	this->telemPacket.gpsLock = this->sensorPacket.gpsLock;
	this->telemPacket.timestamp = this->sensorPacket.timestamp;
	
}

State *State::nextState() {
	return nextState_impl();
}
