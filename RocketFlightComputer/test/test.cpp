#include <ArduinoFake.h>
#include <unity.h>
#include <GNSS.h>
#include <states/Coast.h>
#include <states/Launch.h>
#include <cstdio>
// https://github.com/FabioBatSilva/ArduinoFake

using namespace fakeit;

void setUp() {

}

void tearDown() {

}

void foo_test() {
    Mock<GNSS> mock_gnss;
    When(Method(mock_gnss,getLatitude)).Return(100);

    GNSS& gnss = mock_gnss.get();

    printf("%f\n", gnss.getLatitude());

    State* state = new Coast(gnss);

    state = state->nextState();

    if (state == nullptr) {
        printf("hi\n");
    } else {
        printf("bye\n");
    }

    Verify(Method(mock_gnss,getLatitude)).Once();
    TEST_ASSERT(state != nullptr);
    TEST_ASSERT_EQUAL(state->getId(), StateId::Launch);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();

    RUN_TEST(foo_test);

    return UNITY_END();
}