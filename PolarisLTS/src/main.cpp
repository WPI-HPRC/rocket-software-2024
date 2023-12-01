#include <Arduino.h>
#include <Metro.h>

#include <SPI.h>
#include <Wire.h>

#include <states/State.h>
#include <states/PreLaunch/PreLaunch.h>

// #include <TeensyDebug.h>
// #pragma GCC optimize ("O0")

#include <typeinfo>

SensorFrame sensorFrame;

constexpr static int LOOP_RATE = 100;

unsigned long previousTime = 0;
unsigned long currentTime = 0;
uint32_t counter = 0;

Metro timer = Metro(1000/ LOOP_RATE);

State * state;

Sensorboard sensorBoard;

void setup() {
	Serial.begin(115200);
	// debug.begin(SerialUSB1);

	// halt_cpu();

	Wire.begin();
	Wire.setClock(400000);

	Serial.println("[Polaris] Initializing Sensor Board");
	if(sensorBoard.setup()) {
		Serial.println("[Polaris] Sensor Setup Complete!");
	} else {
		Serial.println("[Polaris] Sensor Setup Failed!");
		while(1) {};
	}

	state = new PreLaunch();

	state->initialize();
	currentTime = millis();
	previousTime = millis();
};

void readSensors() {
	sensorBoard.readInertialSensors();
	memcpy(&sensorFrame, &sensorBoard.Inertial_Baro_frame, sizeof(sensorBoard.Inertial_Baro_frame));
};

void loop() {
	currentTime = millis();

	if(timer.check() == 1) {
		readSensors();

		memcpy(&state->sensorData, &sensorFrame, sizeof(sensorFrame));

		state->loop();

		// Check for state transition each loop
		State *nextState = state->nextState();

		if(nextState != nullptr) {
			Serial.print("State Change Detected: ");
			Serial.print(state->name);
			Serial.print(" -> ");
			state = nextState;
			Serial.println(state->name);

			state->initialize();
		};

		Serial.println(currentTime - previousTime);

		counter++;

		previousTime = currentTime;
	};
};