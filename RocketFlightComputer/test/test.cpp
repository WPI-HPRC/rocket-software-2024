#include <ArduinoFake.h>
#include <unity.h>
// #include <GNSS.h>
#include <Coast.h>
#include <Launch.h>
// https://github.com/FabioBatSilva/ArduinoFake

using namespace fakeit;

void setUp() {

}

void tearDown() {

}

void foo_test() {
    // Mock<GNSS> gnss;
    // When(Method(gnss, getLatitude)).Return(100);

    State* state = new Coast();

    state = state->nextState();

    // Verify(Method(gnss, getLatitude)).Once();
    TEST_ASSERT_EQUAL(state->getId(), StateId::Launch);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();

    RUN_TEST(foo_test);

    return UNITY_END();
}