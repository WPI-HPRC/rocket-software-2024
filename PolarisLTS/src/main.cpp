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

constexpr static int LOOP_RATE = 40;

Metro timer = Metro(1000/ LOOP_RATE);
int counter = 0;
uint32_t timestamp;

long loopStartTime;
long loopTime;
long previousTime;

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

	timer.reset();
	previousTime = millis();
	loopStartTime = millis();

	state = new PreLaunch();

	state->initialize();
};

void readSensors() {
	sensorBoard.readInertialSensors();
	memcpy(&sensorFrame, &sensorBoard.Inertial_Baro_frame, sizeof(sensorBoard.Inertial_Baro_frame));
};

void loop() {
	if(timer.check() == 1) {

		readSensors();

		state->sensorData = sensorFrame;

		state->loop();
		
		State *nextState = state->nextState();

		if(nextState != nullptr) {
			Serial.print("State Change Detected: ");
			// Serial.print(typeid(state).name());
			Serial.print(state->name);
			Serial.print(" -> ");
			state = nextState;
			Serial.println(state->name);
			// Serial.println(typeid(state).name());

			state->initialize();
		};

		counter++;
	};
};