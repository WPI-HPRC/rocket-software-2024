#include <ArduinoFake.h>
#include <unistd.h>
#include <utility.hpp>
#include <cstdio>
#include "EKF/AttitudeEKF.h"
#include "EKF/KinematicEKF.h"
#include "fakeit.hpp"
#include "states/00-PreLaunch.h"
#include "Sensors.h"
#include "unity.h"

using namespace fakeit;
#define FAKEIT_ASSERT_ON_UNEXPECTED_METHOD_INVOCATION

void setUp() {
  ArduinoFakeReset();
  int res = chdir("test");
  TEST_ASSERT_MESSAGE(res != -1, "chdir test failed!");

  When(Method(ArduinoFake(), pinMode)).AlwaysReturn();
  When(Method(ArduinoFake(), digitalWrite)).AlwaysReturn();
  When(Method(ArduinoFake(), delay)).AlwaysReturn();
  When(OverloadedMethod(ArduinoFake(Serial), print, unsigned long (char const *))).AlwaysReturn();
  When(OverloadedMethod(ArduinoFake(Serial), print, unsigned long (const String&))).AlwaysReturn();
  When(OverloadedMethod(ArduinoFake(Serial), println, unsigned long (char const *))).AlwaysReturn();
  When(OverloadedMethod(ArduinoFake(Serial), println, unsigned long (long, int))).AlwaysReturn();
  When(OverloadedMethod(ArduinoFake(Serial), begin, void (unsigned long))).AlwaysReturn();
  When(OverloadedMethod(ArduinoFake(SPI), transfer, unsigned char (unsigned char))).AlwaysReturn();
}

void tearDown() {
  (void)chdir("..");
}

int parseLine(FILE *dataFile, Utility::SensorPacket *packet) {
  int read = 0;
  read += fscanf(dataFile, "%f,", &packet->accelX);
  read += fscanf(dataFile, "%f,", &packet->accelY);
  read += fscanf(dataFile, "%f,", &packet->accelZ);
  read += fscanf(dataFile, "%f,", &packet->gyroX);
  read += fscanf(dataFile, "%f,", &packet->gyroY);
  read += fscanf(dataFile, "%f,", &packet->gyroZ);
  read += fscanf(dataFile, "%f,", &packet->magX);
  read += fscanf(dataFile, "%f,", &packet->magY);
  read += fscanf(dataFile, "%f,", &packet->magZ);
  read += fscanf(dataFile, "%f,", &packet->pressure);
  read += fscanf(dataFile, "%f,", &packet->temperature);
  read += fscanf(dataFile, "%f,", &packet->gpsLat);
  read += fscanf(dataFile, "%f,", &packet->gpsLong);
  read += fscanf(dataFile, "%f,", &packet->gpsAltMSL);
  read += fscanf(dataFile, "%f,", &packet->gpsAltAGL);
  read += fscanf(dataFile, "%d,", &packet->gpsVelocityN);
  read += fscanf(dataFile, "%d,", &packet->gpsVelocityE);
  read += fscanf(dataFile, "%d,", &packet->gpsVelocityD);
  read += fscanf(dataFile, "%d,", &packet->epochTime);
  read += fscanf(dataFile, "%hhd,", &packet->satellites);
  read += fscanf(dataFile, "%hhd,", (char *)&packet->gpsLock);
  read += fscanf(dataFile, "%du", &packet->timestamp);
  if (read < 0) read = EOF;
  return read;
}

void writeLine(FILE *outFile, Utility::TelemPacket packet) {
    fprintf(outFile, "%hhd,", packet.state);
    fprintf(outFile, "%f,", packet.accelX);
    fprintf(outFile, "%f,", packet.accelY);
    fprintf(outFile, "%f,", packet.accelZ);
    fprintf(outFile, "%f,", packet.gyroX);
    fprintf(outFile, "%f,", packet.gyroY);
    fprintf(outFile, "%f,", packet.gyroZ);
    fprintf(outFile, "%f,", packet.rawMagX);
    fprintf(outFile, "%f,", packet.rawMagY);
    fprintf(outFile, "%f,", packet.rawMagZ);
    fprintf(outFile, "%f,", packet.pressure);
    fprintf(outFile, "%f,", packet.temperature);
    fprintf(outFile, "%d,", packet.servoPosition);
    fprintf(outFile, "%f,", packet.altitude);
    fprintf(outFile, "%f,", packet.magX);
    fprintf(outFile, "%f,", packet.magY);
    fprintf(outFile, "%f,", packet.magZ);
    fprintf(outFile, "%f,", packet.w);
    fprintf(outFile, "%f,", packet.i);
    fprintf(outFile, "%f,", packet.j);
    fprintf(outFile, "%f,", packet.k);
    fprintf(outFile, "%f,", packet.posX);
    fprintf(outFile, "%f,", packet.posY);
    fprintf(outFile, "%f,", packet.posZ);
    fprintf(outFile, "%f,", packet.velX);
    fprintf(outFile, "%f,", packet.velY);
    fprintf(outFile, "%f,", packet.velZ);
    fprintf(outFile, "%f,", packet.gpsLat);
    fprintf(outFile, "%f,", packet.gpsLong);
    fprintf(outFile, "%f,", packet.gpsAltMSL);
    fprintf(outFile, "%f,", packet.gpsAltAGL);
    fprintf(outFile, "%d,", packet.epochTime);
    fprintf(outFile, "%hhd,", packet.satellites);
    fprintf(outFile, "%hhd,", packet.gpsLock);
    fprintf(outFile, "%d,", packet.loopCount);
    fprintf(outFile, "%d\n", packet.timestamp);
}

void test_basic() {
  FILE * dataFile = fopen("mock_data_basic.csv", "r");
  TEST_ASSERT_MESSAGE(dataFile != nullptr, "No data file found (mock_data_basic.csv)");
  FILE * outFile = fopen("mock_data_basic_telem.csv", "w");

  Utility::SensorPacket data;

  Mock<Sensors> mockSensors;
  AttitudeStateEstimator *attitude_ekf = new AttitudeStateEstimator();
  KinematicStateEstimator *kinematic_ekf = new KinematicStateEstimator();

  When(Method(ArduinoFake(), millis)).AlwaysDo([&data](){ return data.timestamp; });
  When(OverloadedMethod(mockSensors, readSensors, Utility::SensorPacket ())).AlwaysDo([&data](){ return data; });

  Sensors &sensors = mockSensors.get();

  State *state = new PreLaunch(&sensors, attitude_ekf,kinematic_ekf);
  state->initialize();

  int n;
  while ((n = parseLine(dataFile, &data)) == 22) {
    state->loop();
    writeLine(outFile, state->telemPacket);
    State *nextState = state->nextState();
    if (nextState != nullptr) {
      delete state;
      state = nextState;
      state->initialize();
    }
  }
  TEST_ASSERT_MESSAGE(n == EOF, "Wrong number of data points in packet");
}

void irec2023() {
  FILE * dataFile = fopen("mock_data_irec2023.csv", "r");
  TEST_ASSERT_MESSAGE(dataFile != nullptr, "No data file found (mock_data_irec2023.csv)");
  FILE * outFile = fopen("mock_data_irec2023_telem.csv", "w");

  Utility::SensorPacket data;

  Mock<Sensors> mockSensors;
  AttitudeStateEstimator *attitude_ekf = new AttitudeStateEstimator();
  KinematicStateEstimator *kinematic_ekf = new KinematicStateEstimator();

  When(Method(ArduinoFake(), millis)).AlwaysDo([&data](){ return data.timestamp; });
  When(OverloadedMethod(mockSensors, readSensors, Utility::SensorPacket ())).AlwaysDo([&data](){ return data; });

  Sensors &sensors = mockSensors.get();

  State *state = new PreLaunch(&sensors, attitude_ekf,kinematic_ekf);
  state->initialize();

  int n;
  while ((n = parseLine(dataFile, &data)) == 22) {
    state->loop();
    writeLine(outFile, state->telemPacket);
    State *nextState = state->nextState();
    if (nextState != nullptr) {
      delete state;
      state = nextState;
      state->initialize();
    }
  }
  TEST_ASSERT_MESSAGE(n == EOF, "Wrong number of data points in packet");
}

int main(int argc, char** argv) {
  UNITY_BEGIN();

  RUN_TEST(test_basic);
  RUN_TEST(irec2023);

  return UNITY_END();
}
