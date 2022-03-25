#include <Arduino.h>
#include <unity.h>
#include <Range.h>
#include <PidController.h>

/*
// Mocks out the millis function from the arduino library
// so that these tests can be run on a desktop machine
unsigned long time = 0;
unsigned long millis()
{
    time += 1;
    return time;
}
*/

void test_function_kp(void)
{
    const Range inputRange = Range(0, 100);
    const Range outputRange = Range(0, 100);
    const PidControllerConfig pidConfig = PidControllerConfig(
        1.0, 0.0, 0.0, inputRange, outputRange);
    PidController pid(pidConfig);
    TEST_ASSERT_EQUAL(pid.update(50, 0), 50);
    TEST_ASSERT_EQUAL(pid.update(50, 100), 0);

    pid.pidConfig.kp = 2.0;
    TEST_ASSERT_EQUAL(pid.update(50, 0), 100);
    TEST_ASSERT_EQUAL(pid.update(50, 100), 0);
}

void test_function_clamps(void)
{
    const Range inputRange = {0, 100};
    const Range outputRange = {0, 100};
    const PidControllerConfig pidConfig = PidControllerConfig(
        0.5, 0.0, 0.0, inputRange, outputRange);
    PidController pid(pidConfig);
    TEST_ASSERT_EQUAL(pid.update(50, 0), 25);
    TEST_ASSERT_EQUAL(pid.update(50, 100), 25);

    pid.pidConfig.kp = 10.0;
    TEST_ASSERT_EQUAL(pid.update(50, 0), 100);
    TEST_ASSERT_EQUAL(pid.update(50, 100), 0);
}

void setup()
{
    // NOTE!!! Wait for >2 secs
    // if board doesn't support software reset via Serial.DTR/RTS
    delay(2000);

    UNITY_BEGIN(); // IMPORTANT LINE!

    pinMode(LED_BUILTIN, OUTPUT);
}

// https://docs.platformio.org/en/latest/tutorials/core/unit_testing_blink.html
void loop()
{

    // Todo: Fix the tests
    // RUN_TEST(test_function_kp);
    // RUN_TEST(test_function_clamps);

    while (true)
    {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(100);
        digitalWrite(LED_BUILTIN, LOW);
        delay(100);
        UNITY_END(); // stop unit testing
    }
}
