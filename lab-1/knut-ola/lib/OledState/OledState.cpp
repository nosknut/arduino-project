#include "OledState.h"

OledState getNextOledState(OledState oled)
{
    // use static_cast to convert from enum to int
    int currentStateIndex = static_cast<int>(oled);
    if (currentStateIndex < (OLED_STATE_LENGTH - 1))
    {
        int nextStateIndex = currentStateIndex + 1;
        // use static_cast to convert from int to enum
        return static_cast<OledState>(nextStateIndex);
    }
    int firstStateIndex = 0;
    // use static_cast to convert from int to enum
    return static_cast<OledState>(firstStateIndex);
}
