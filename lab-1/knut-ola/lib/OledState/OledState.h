#ifndef OledState_h
#define OledState_h

enum class OledState
{
    DISTANCE,
    TEMPERATURE,
    LIGHT
};

const int OLED_STATE_LENGTH = 3;

OledState getNextOledState(OledState oled);

#endif
