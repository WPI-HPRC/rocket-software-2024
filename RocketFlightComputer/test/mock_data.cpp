#include <ArduinoFake.h>
#include <unistd.h>
#include <utility.hpp>
#include <cstdio>
#include "EKF/EKF.h"
#include "fakeit.hpp"
#include "states/00-PreLaunch.h"
#include "Sensors.h"
#include "unity.h"

using namespace fakeit;

void setUp() {
  ArduinoFakeReset();
  int res = chdir("test");
  TEST_ASSERT_MESSAGE(res != -1, "chdir test failed!");
}

int parseLine(FILE *dataFile, Utility::SensorPacket *packet) {
  return fscanf(dataFile, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%hhd,%hhd,%d",
              &packet->accelX, &packet->accelY, &packet->accelZ,
              &packet->gyroX, &packet->gyroY, &packet->gyroZ,
              &packet->magX, &packet->magY, &packet->magZ,
              &packet->pressure,
              &packet->gpsLat, &packet->gpsLong, &packet->gpsAltMSL, &packet->gpsAltAGL,
              &packet->epochTime, &packet->satellites, (char *)&packet->gpsLock, &packet->timestamp);
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
  StateEstimator *ekf = new StateEstimator(BLA::Matrix<10> {1, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 0.25);

  When(Method(ArduinoFake(), millis)).AlwaysDo([&data](){ return data.timestamp; });
  When(Method(ArduinoFake(), pinMode)).AlwaysReturn();
  When(Method(ArduinoFake(), digitalWrite)).AlwaysReturn();
  When(Method(ArduinoFake(), delay)).AlwaysReturn();
  When(OverloadedMethod(ArduinoFake(Serial), print, unsigned long (char const *))).AlwaysReturn();
  When(OverloadedMethod(ArduinoFake(Serial), println, unsigned long (char const *))).AlwaysReturn();
  When(OverloadedMethod(ArduinoFake(Serial), println, unsigned long (long, int))).AlwaysReturn();
  When(OverloadedMethod(ArduinoFake(Serial), begin, void (unsigned long))).AlwaysReturn();
  When(OverloadedMethod(ArduinoFake(SPI), transfer, unsigned char (unsigned char))).AlwaysReturn();
  When(OverloadedMethod(mockSensors, readSensors, Utility::SensorPacket ())).AlwaysDo([&data](){ return data; });

  Sensors &sensors = mockSensors.get();

  State *state = new PreLaunch(&sensors, ekf);
  state->initialize();

  int n;
  while ((n = parseLine(dataFile, &data)) == 18) {
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

  return UNITY_END();
}
