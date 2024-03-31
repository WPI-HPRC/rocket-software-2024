#include <ArduinoFake.h>
#include "unity.h"

using namespace fakeit;

void setUp() {
  ArduinoFakeReset();
}

void test_basic() {
}

int main(int argc, char** argv) {
  UNITY_BEGIN();

  RUN_TEST(test_basic);

  return UNITY_END();
}
