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
    Utility::logData(this->flash, this->sensorPacket);
	loop_impl();
	this->lastLoopTime = millis();
}

State *State::nextState() {
	return nextState_impl();
}
