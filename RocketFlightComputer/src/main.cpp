#include <Arduino.h>
#include <lib/MetroTimer/Metro.h>
#include <Wire.h>
#include <SPI.h>


#define LOOP_RATE 100

Metro timer = Metro(1000 / LOOP_RATE);
int counter = 0;
uint32_t timestamp;

long loopStartTime;
long loopTime;
long previousTime;

enum RocketState { 
  STARTUP,
  LOOP
};

RocketState state = STARTUP;

void setup() {
  Serial.begin(115200); 

  Wire.begin();
  Wire.setClock(400000);

  timer.reset();
  previousTime = millis();
  loopStartTime = millis();

  state = LOOP;
};

void readSensors() {

};

void loop() {
  
  if(timer.check() == 1) {
    switch (state)
    {
      case STARTUP:
        Serial.println("+=HPRC FLIGHT COMPUTER=+");
        Serial.println("Starting up...");
        /* code */
        break;
      case LOOP:
        readSensors();

        break;
    };

    counter++;
  }
};