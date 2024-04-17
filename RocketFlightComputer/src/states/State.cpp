#include "State.h"
#include "utility.hpp"
#include <Arduino.h>

State::State(struct Sensors *sensors, StateEstimator *stateEstimator) : sensors(sensors), stateEstimator(stateEstimator) {}

void break_uint16(uint16_t value, uint8_t *byte_array) {
    byte_array[1] = (uint8_t)(value & 0xFF);        // Low byte
    byte_array[0] = (uint8_t)((value >> 8) & 0xFF); // High byte
}

void State::initialize()
{
    this->startTime = millis();
    initialize_impl();

    xbee->start();

/*

    const uint16_t networkID = 0x4843;

    uint8_t byte_array[2];
    
    break_uint16(networkID, byte_array);

    xbee->setParameter(XBee::AtCommand::ApiOptions, 0x90);
    xbee->setParameter(AsciiToUint16('I', 'D'), byte_array, 2);
*/

    // exit(0);
}

void State::loop() {
	long now = millis();
  // These values may be used in the state code
	this->currentTime = now - this->startTime;
	this->deltaTime = now - this->lastLoopTime;
	this->loopCount++;
	this->sensorPacket = this->sensors->readSensors();
  this->telemPacket.altitude = Utility::pressureToAltitude(this->sensorPacket.pressure);
  this->telemPacket.servoPosition = analogRead(SERVO_FEEDBACK_GPIO);
	if (this->stateEstimator->initialized) {
		this->stateEstimator->onLoop(this->sensorPacket);

		this->telemPacket.w = this->stateEstimator->x(0);
		this->telemPacket.i = this->stateEstimator->x(1);
		this->telemPacket.j = this->stateEstimator->x(2);
		this->telemPacket.k = this->stateEstimator->x(3);
	}
	
	loop_impl();
	this->lastLoopTime = now;
  // These values are for logging, and should not be used in the state code without moving the corresponding line above loop_impl()
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

  this->telemPacket.gpsLat = this->sensorPacket.gpsLat;
  this->telemPacket.gpsLong = this->sensorPacket.gpsLong;
  this->telemPacket.epochTime = this->sensorPacket.epochTime;
  this->telemPacket.satellites = this->sensorPacket.satellites;
  this->telemPacket.gpsLock = this->sensorPacket.gpsLock;
  this->telemPacket.loopCount = this->loopCount;
  this->telemPacket.timestamp = now;

  if (sdCardInitialized) {
    dataFile.write((uint8_t *)&this->telemPacket, sizeof(this->telemPacket));
  }

#ifdef SERIAL_TELEMETRY
    this->telemPacket.debugPrint();
#endif

    /** Loop Radio and Send Data */

    Serial.printf("Loop count: %llu\n", this->loopCount);

#ifndef NO_XBEE

    xbee->sendTransmitRequestCommand(0x0013A200423F474C, (uint8_t *)&telemPacket, sizeof(telemPacket));

    // Serial.print("Packet Success: ");
    // Serial.println(now);
#endif
}

State *State::nextState()
{
#ifndef NO_TRANSITION
    return nextState_impl();
#else
		return nullptr;
#endif
}
