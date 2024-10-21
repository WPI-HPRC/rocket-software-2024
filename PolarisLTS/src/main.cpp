#include "EKF/AttitudeEKF.h"
#include <Arduino.h>
#include <Metro.h>

#include <SPI.h>
#include <Wire.h>

#include <SD.h>

#include <states/00-PreLaunch.h>
#include <states/State.h>

#include <SensorBoardLibraries/SensorBoard.hpp>

// #include <TeensyDebug.h>
// #pragma GCC optimize ("O0")

SensorFrame sensorFrame;

// FlashChip flash = FlashChip();

uint64_t previousTime = 0;
uint64_t currentTime = 0;

State *state;

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
XbeeProSX xbee = XbeeProSX(30); // CS 30
#endif

void setup() {
  Serial.begin(9600);

#ifdef WAIT_FOR_SERIAL
  while (!Serial) {
    yield();
  }
#endif
  Wire.begin();
  Wire.setClock(400000);

  SPI.begin();

#ifndef NO_SDCARD
  if (SD.begin(31)) {
    int fileIdx = 0;
    while (1) {
      char filename[100];
      sprintf(filename, "flightData%d.bin", fileIdx++);
      Serial.printf("Trying file `%s`\n", filename);
      if (!SD.exists(filename)) {
        dataFile = SD.open(filename, O_WRONLY | O_CREAT);
        break;
      }
    }
    sdCardInitialized = true;
  } else {
    Serial.println("SD Init failed");
  }
#endif

  Serial.println("[Polaris] Initializing Sensor Board");
  if (sensorBoard.setup()) {
    Serial.println("[Polaris] Sensor Setup Complete!");
  } else {
    Serial.println("[Polaris] Sensor Setup Failed!");
    while (1) {
    };
  }

  pinMode(6, OUTPUT);
  digitalWrite(6, HIGH);

  state = (State *)new PreLaunch(&sensorBoard, attitudeStateEstimator);

  state->initialize();

#ifndef NO_XBEE
  xbee.start();
#endif

  // flash.init();
  // int startAddress = 0;
  // startAddress = flash.rememberAddress();
  // Serial.println("Starting Flash Chip At Address: " + String(startAddress));
};

// void readSensors() {
// 	sensorBoard.readInertialSensors();
// 	memcpy(&sensorFrame, &sensorBoard.Inertial_Baro_frame,
// sizeof(sensorBoard.Inertial_Baro_frame));
// };

bool val = false;
long lastBlink = 0;

void loop() {
  currentTime = millis();
  if (currentTime - lastBlink >= 1000) {
    lastBlink = currentTime;
    val = !val;
    if (sdCardInitialized) {
      digitalWrite(6, HIGH);
    } else {
      digitalWrite(6, val);
    }
  }
  if (currentTime - previousTime >= (1000 / LOOP_RATE)) {
    previousTime = currentTime;
    state->loop();

    // String timestamp = (String) millis();

    // String structString = String(state->telemPacket.accelX) + "," +
    // String(state->telemPacket.accelY) + "," +
    // String(state->telemPacket.accelZ) + "," +
    // String(state->telemPacket.gyroX) + "," + String(state->telemPacket.gyroY)
    // + "," + String(state->telemPacket.gyroZ) + "," +
    // String(state->telemPacket.magX) + "," + String(state->telemPacket.magY) +
    // "," + String(state->telemPacket.magZ) + "," +
    // String(state->telemPacket.pressure) + "," +
    // String(state->telemPacket.altitude) + "," + String(state->telemPacket.q)
    // + "," + String(state->telemPacket.i) + "," + String(state->telemPacket.j)
    // + "," + String(state->telemPacket.k) + "," + timestamp;

    // Serial.println(structString);

    // flash.writeStruct(structString);

    // Check for state transition each loop
    State *nextState = state->nextState();

    if (nextState != nullptr) {
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
