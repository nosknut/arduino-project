#include <unity.h>
#include <OledState.h>

const OledState last = OledState::LIGHT;

void test_function_getNextOledState_findsNext(void)
{
    const OledState current = OledState::DISTANCE;
    const OledState next = OledState::TEMPERATURE;
    TEST_ASSERT_EQUAL(next, getNextOledState(current));
}

void test_function_getNextOledState_cylesOnLast(void)
{
    const OledState first = static_cast<OledState>(0);
    TEST_ASSERT_EQUAL(first, getNextOledState(last));
}

// Ensures that the OLED_STATE_LENGTH variable is always up to date
void test_function_OledState_lengthVariableMatchesEnumLength(void)
{
    const int lastIndex = static_cast<int>(last);
    TEST_ASSERT_EQUAL(OLED_STATE_LENGTH, lastIndex + 1);
    //"Please update the OLED_STATE_LENGTH variable in OledState.h to represent the number of values in the enum";
}

// https://docs.platformio.org/en/latest/plus/unit-testing.html
int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_function_getNextOledState_findsNext);
    RUN_TEST(test_function_getNextOledState_cylesOnLast);
    RUN_TEST(test_function_OledState_lengthVariableMatchesEnumLength);
    UNITY_END();

    return 0;
}
