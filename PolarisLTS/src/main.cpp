#include "EKF/AttitudeEKF.h"
#include <Arduino.h>
#include <Metro.h>

#include <SPI.h>
#include <Wire.h>

#include <states/State.h>
#include <states/00-PreLaunch.h>

#include <SensorBoardLibraries/SensorBoard.hpp>

// #include <TeensyDebug.h>
// #pragma GCC optimize ("O0")

SensorFrame sensorFrame;

// FlashChip flash = FlashChip();

uint64_t previousTime = 0;
uint64_t currentTime = 0;

State * state;

Sensorboard sensorBoard;
AttitudeStateEstimator *attitudeStateEstimator = new AttitudeStateEstimator();

#ifndef NO_SDCARD
bool sdCardInitialized = false;
File dataFile;
#endif

#ifndef NO_SERVO
Servo airbrakesServo = Servo();
#endif

#ifndef NO_XBEE
XbeeProSX xbee = XbeeProSX(17); // CS GPIO17
#endif

void setup() {
	Serial.begin(115200);

	while(!Serial);
	Wire.begin();
	Wire.setClock(400000);

	Serial.println("[Polaris] Initializing Sensor Board");
	if(sensorBoard.setup()) {
		Serial.println("[Polaris] Sensor Setup Complete!");
	} else {
		Serial.println("[Polaris] Sensor Setup Failed!");
		while(1) {};
	}

	state = (State *)new PreLaunch(&sensorBoard, attitudeStateEstimator);

	state->initialize();

	// flash.init();
	// int startAddress = 0;
	// startAddress = flash.rememberAddress();
	// Serial.println("Starting Flash Chip At Address: " + String(startAddress));
};

// void readSensors() {
// 	sensorBoard.readInertialSensors();
// 	memcpy(&sensorFrame, &sensorBoard.Inertial_Baro_frame, sizeof(sensorBoard.Inertial_Baro_frame));
// };

void loop() {
	currentTime = millis();
	if(currentTime - previousTime >= (1000 / LOOP_RATE)) {
		state->loop();

		// String timestamp = (String) millis();

		// String structString = String(state->telemPacket.accelX) + "," + String(state->telemPacket.accelY) + "," + String(state->telemPacket.accelZ) + "," + String(state->telemPacket.gyroX) + "," + String(state->telemPacket.gyroY) + "," + String(state->telemPacket.gyroZ) + "," + String(state->telemPacket.magX) + "," + String(state->telemPacket.magY) + "," + String(state->telemPacket.magZ) + "," + String(state->telemPacket.pressure) + "," + String(state->telemPacket.altitude) + "," + String(state->telemPacket.q) + "," + String(state->telemPacket.i) + "," + String(state->telemPacket.j) + "," + String(state->telemPacket.k) + "," + timestamp;

		// Serial.println(structString);

		// flash.writeStruct(structString);

		// Check for state transition each loop
		State *nextState = state->nextState();

		if(nextState != nullptr) {
			Serial.print("State Change Detected: ");
			Serial.print(state->getId());
			Serial.print(" -> ");
			delete state;
			state = nextState;
			Serial.println(state->getId());

			state->initialize();
		};
	};
};
