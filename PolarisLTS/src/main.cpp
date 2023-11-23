#include <Arduino.h>
#include <Metro.h>
#include <typeinfo>

#include <SPI.h>
#include <Wire.h>

#include <states/State.h>
#include <states/PreLaunch/PreLaunch.h>

constexpr static int LOOP_RATE = 40;

Metro timer = Metro(1000/ LOOP_RATE);
int counter = 0;
uint32_t timestamp;

long loopStartTime;
long loopTime;
long previousTime;

State *state = new PreLaunch();

void setup() {
	Serial.begin(115200);

	while(!Serial);

	Wire.begin();
	Wire.setClock(400000);

	timer.reset();
	previousTime = millis();
	loopStartTime = millis();

	state->initialize();
};

void readSensors() {

};

void loop() {
	if(timer.check() == 1) {
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