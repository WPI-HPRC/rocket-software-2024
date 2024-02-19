#include "State.h"
#include <Arduino.h>

State::State(struct Sensors *sensors, StateEstimator *stateEstimator) : sensors(sensors), stateEstimator(stateEstimator) {}

void State::initialize()
{
    this->startTime = millis();
    initialize_impl();

    xbee->begin();
}

void State::loop() {
	long long now = millis();
	this->currentTime = now - this->startTime;
	this->deltaTime = now - this->lastLoopTime;
	this->loopCount++;
	this->sensorPacket = this->sensors->readSensors();
	if (this->stateEstimatorInitialized) {
		this->x_state = this->stateEstimator->onLoop(this->sensorPacket);

		this->telemPacket.w = this->x_state(0);
		this->telemPacket.i = this->x_state(1);
		this->telemPacket.j = this->x_state(2);
		this->telemPacket.k = this->x_state(3);
	}
	
	loop_impl();
	this->lastLoopTime = millis();
	
	/**
	 * Assemble Telemetry packet from sensor packet, this is stuff we want done every loop
	*/
  this->telemPacket.state = this->getId();
	this->telemPacket.accelX = this->sensorPacket.accelX;
	this->telemPacket.accelY = this->sensorPacket.accelY;
	this->telemPacket.accelZ = this->sensorPacket.accelZ;

    /**
     * Assemble Telemetry packet from sensor packet, this is stuff we want done every loop
     */
    this->telemPacket.state = this->getId();
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
    this->telemPacket.timestamp = millis();

    /** Loop Radio and Send Data */
    xbee->updateSubscribers();

    xbee->send(0x0013A200423F474C, &telemPacket, sizeof(telemPacket));

    // Serial.print("Packet Success: ");
    // Serial.println(millis());

    // Serial.print("Packet Size: "); Serial.println(sizeof(telemPacket));
    // Serial.print("Data Packet Size: "); Serial.println(sizeof(telemPacket));
}

State *State::nextState()
{
    return nextState_impl();
}
