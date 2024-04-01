#include <ArduinoFake.h>
#include <unistd.h>
#include <utility.hpp>
#include <cstdio>
#include "unity.h"

using namespace fakeit;

void setUp() {
  ArduinoFakeReset();
  (void)chdir("test");
}

int parse_line(FILE *data_file, Utility::SensorPacket *packet) {
  return fscanf(data_file, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%d,%d,%d",
              &packet->accelX, &packet->accelY, &packet->accelZ,
              &packet->gyroX, &packet->gyroY, &packet->gyroZ,
              &packet->magX, &packet->magY, &packet->magZ,
              &packet->pressure, &packet->altitude,
              &packet->gpsLat, &packet->gpsLong, &packet->gpsAltMSL, &packet->gpsAltAGL,
              &packet->epochTime, &packet->satellites, &packet->gpsLock, &packet->timestamp);
}

void test_basic() {
  FILE * data_file = fopen("mock_data_basic.csv", "r");
  TEST_ASSERT_MESSAGE(data_file != nullptr, "No data file found (mock_data_basic.csv)");

  Utility::SensorPacket data;
  TEST_ASSERT_EQUAL_MESSAGE(19, parse_line(data_file, &data), "Wrong number of data points for packet");
}

int main(int argc, char** argv) {
  UNITY_BEGIN();

  RUN_TEST(test_basic);

  return UNITY_END();
}
