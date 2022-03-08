#include <PidControllerConfig.h>
#include <Range.h>

// NUM_LINE_SENSORS must be defined as a const separately
// of appConfig because it is needed when declaring an array
#define APP_CONFIG_NUM_LINE_SENSORS 5

struct ApplicationConfig
{
    const int baudRate = 9600;
    const int lcdWidth = 8;

    // Speed
    const int targetSpeed = 200;
    const int calibrationSpeed = 200;
    const int maxSpeed = 300;

    // All LEDS will flash x times before the motors start
    const int numSafetyFlashesBeforeStart = 5;
    const int safetyFlashDurationMs = 400;

    // Pid
    PidControllerConfig linePidConfig =
        PidControllerConfig(
            2.0,                       // kp
            0.0,                       // ki
            0.0,                       // kd
            Range(0, 4000),            // inputRange
            Range(-maxSpeed, maxSpeed) // outputRange
        );
} const appConfig;
